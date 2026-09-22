#!/usr/bin/env python3
"""UWB tag -> MQTT range bridge, for the unified uwb_tdma_twr firmware.

Runs on the Pi the tags are physically plugged into. Auto-discovers which
serial devices are tags (as opposed to anchors, or something else on the
bus entirely) by querying each candidate's own $CFG and matching its
node_id against the known fleet -- not by hardcoding /dev/ttyUSB* device
paths or USB serial numbers, both of which change with plug order and
don't survive a board being swapped for a spare. A device that never
answers $CFG, or answers with an unknown node_id, or answers as an anchor
(role=RESP) rather than a tag, is left alone.

Each tag then gets one dedicated reader thread, decoding its live $RNG
stream and publishing one MQTT message per range -- anchor misses
(RNG_FLAG_RESP_LOST, the synthetic 0x0000 placeholder line) are dropped,
not published, same as every other tool in this repo's tools/ that reads
$RNG.

    topic:   uwb/ranging/<tag_label>/<anchor_label>
    payload: {"tag_label":"T1","anchor_label":"A1","range_m":1.234,
              "rssi":-77.0,"timestamp_sent":647306867,
              "timestamp_received":1758540000.123456}

timestamp_sent is the tag's own t_local_us field from the $RNG line (its
free-running microsecond counter, wraps every ~71.6 min -- a delta within
one tag's stream, not a wall-clock value). timestamp_received is this
bridge's host wall-clock time.time() when the line was parsed. Same
convention as tools/raw_json.py on the Mac side, so a payload from either
is drop-in comparable.

Reconnects a tag's serial port automatically if it drops (unplug/replug,
USB enumeration hiccup) rather than killing that thread -- an anchor or a
totally unrelated USB device continues to be ignored, not retried, since
a periodic rescan (--rescan-every, default 15s) is what discovers genuinely
new devices.
"""
import argparse
import glob
import json
import queue
import re
import sys
import threading
import time

import serial
import paho.mqtt.client as mqtt

# Known fleet identities: node_id (as $CFG prints it) -> label. Update this
# alongside tools/uwb.py's BOARDS/fleet.py's provisioning if the fleet
# changes -- this file has no other source of truth for who's who.
TAGS = {
    "0x8052": "T1", "0x8053": "T2", "0x4051": "T3",
    "0x7851": "T4", "0x0104": "T5", "0x0105": "T6",
}
ANCHORS = {
    "0x0101": "A1", "0x0F70": "A2", "0x0102": "A3", "0x0E5B": "A4",
    "0x0103": "A5", "0x0106": "A6", "0xC050": "A7",
}

# ESP32 (CP2104) tags run at 921600; DWM1001 (J-Link VCOM) tags run at
# 115200 -- see tools/uwb.py's BOARDS comment for why the split is safe
# (baud is per-board, not part of cfg_hash). Tried in this order since
# there are more ESP32s in the fleet right now.
PROBE_BAUDS = (921600, 115200)

RNG_FLAG_RESP_LOST = 0x04


def cs(payload):
    """$SET/$GET/$CFG framing with the firmware's XOR checksum -- copied
    verbatim from tools/uwb.py's cs() so this and the Mac-side tools always
    agree on the wire format, since a formula only kept in two places will
    eventually drift."""
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return f"\r\n${payload}*{c:02X}\r\n".encode()


def check_line(line):
    """Validate one '$PAYLOAD*XX' line. Returns the payload (no '$', no
    '*XX') or None. Copied verbatim from tools/uwb.py's check_line() --
    see that copy's docstring for why this matters on a link that can tear
    a line under load."""
    if not line or "$" not in line or "*" not in line:
        return None
    line = line[line.rindex("$"):]
    body, _, cks = line[1:].rpartition("*")
    if not body or len(cks) < 2:
        return None
    try:
        want = int(cks[:2], 16)
    except ValueError:
        return None
    got = 0
    for ch in body:
        got ^= ord(ch)
    return body if got == want else None


def candidate_ports():
    """USB-serial devices that might be a tag. by-id first (stable across
    replugs, carries the CP2104/J-Link serial number in its name for
    logging) -- ttyUSB*/ttyACM* as a fallback for a device with no by-id
    symlink yet."""
    seen, out = set(), []
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        out.append(p)
        seen.add(p)
    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for p in sorted(glob.glob(pat)):
            # Skip anything already reachable via its by-id symlink target.
            import os
            try:
                real = os.path.realpath(p)
            except OSError:
                continue
            if not any(os.path.realpath(s) == real for s in seen):
                out.append(p)
    return out


def identify(port, timeout=1.5):
    """Open port, ask $CFG, return (label, baud) if it's a known tag, else
    None. Tries each baud in PROBE_BAUDS; a wrong baud reads noise that
    fails check_line() and simply times out to the next baud, not a crash.
    """
    for baud in PROBE_BAUDS:
        try:
            ser = serial.Serial(port, baud, timeout=timeout)
        except (serial.SerialException, OSError):
            continue
        try:
            ser.reset_input_buffer()
            ser.write(cs("CFG"))
            ser.flush()
            deadline = time.time() + timeout
            buf = b""
            role, node_id = None, None
            while time.time() < deadline:
                chunk = ser.read(512)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    body = check_line(raw.decode("ascii", "replace").strip())
                    if body is None:
                        continue
                    if body.startswith("C,role,"):
                        role = body.split(",")[2]
                    elif body.startswith("C,node_id,"):
                        node_id = body.split(",")[2]
                    elif body.startswith("CFGEND"):
                        break
                if role and node_id:
                    break
            if role == "INIT" and node_id in TAGS:
                return TAGS[node_id], baud
            if node_id:
                # A real anchor, or a tag/anchor not in our fleet table --
                # either way, not something this bridge reads from.
                return None
        finally:
            ser.close()
    return None


class TagReader(threading.Thread):
    """One thread per tag: owns its serial port, decodes $RNG, pushes
    parsed ranges onto the shared publish queue. Reconnects on its own if
    the port drops; does not affect any other tag's thread."""

    def __init__(self, label, port, baud, out_q, stop_evt):
        super().__init__(name=f"reader-{label}", daemon=True)
        self.label, self.port, self.baud = label, port, baud
        self.out_q, self.stop_evt = out_q, stop_evt

    def run(self):
        buf = b""
        while not self.stop_evt.is_set():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.5)
            except (serial.SerialException, OSError) as e:
                print(f"[{self.label}] open failed ({e}); retrying in 3s", file=sys.stderr)
                time.sleep(3)
                continue
            print(f"[{self.label}] streaming from {self.port} @ {self.baud}")
            try:
                while not self.stop_evt.is_set():
                    chunk = ser.read(512)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        self._on_line(raw)
            except (serial.SerialException, OSError) as e:
                print(f"[{self.label}] lost port ({e}); reconnecting", file=sys.stderr)
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
            if not self.stop_evt.is_set():
                time.sleep(2)

    def _on_line(self, raw):
        received_at = time.time()
        line = raw.decode("ascii", "replace").strip()
        if not line.startswith("$RNG"):
            return
        body = check_line(line)
        if body is None:
            return
        f = body.split(",")
        # $RNG,<n>,<frame_idx>,<tag_id>,<anchor_id>,<slot>,<d_mm>,<tof_ps>,
        #      <rxp_dbm>,<fpp_dbm>,<flags>,<t_local_us>
        if len(f) != 12 or f[0] != "RNG":
            return
        try:
            anchor_id = f[4]
            d_mm = float(f[6])
            rxp_dbm = float(f[8])
            flags = int(f[10], 16)
            t_local_us = int(f[11])
        except (ValueError, IndexError):
            return
        if flags & RNG_FLAG_RESP_LOST:
            return  # the miss placeholder, not a real range -- see output.h
        anchor_label = ANCHORS.get(anchor_id, anchor_id)  # fall back to raw id if unknown
        self.out_q.put({
            "tag_label": self.label,
            "anchor_label": anchor_label,
            "range_m": round(d_mm / 1000.0, 4),
            "rssi": rxp_dbm,
            "timestamp_sent": t_local_us,
            "timestamp_received": received_at,
        })


def publisher(out_q, stop_evt, host, port):
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(host, port, keepalive=30)
    client.loop_start()
    print(f"connected to mqtt://{host}:{port}")
    try:
        while not stop_evt.is_set():
            try:
                msg = out_q.get(timeout=0.5)
            except queue.Empty:
                continue
            topic = f"uwb/ranging/{msg['tag_label']}/{msg['anchor_label']}"
            client.publish(topic, json.dumps(msg), qos=0)
    finally:
        client.loop_stop()
        client.disconnect()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mqtt-host", default="192.168.0.180")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--rescan-every", type=float, default=15.0,
                     help="seconds between scans for newly-connected tags")
    args = ap.parse_args()

    stop_evt = threading.Event()
    out_q = queue.Queue(maxsize=10000)
    pub_thread = threading.Thread(
        target=publisher, args=(out_q, stop_evt, args.mqtt_host, args.mqtt_port),
        daemon=True)
    pub_thread.start()

    claimed_ports = {}   # port path -> TagReader
    claimed_labels = set()

    try:
        while True:
            for port in candidate_ports():
                if port in claimed_ports:
                    continue
                ident = identify(port)
                if ident is None:
                    continue
                label, baud = ident
                if label in claimed_labels:
                    # Same tag reachable via two device nodes (by-id +
                    # ttyUSB alias) -- keep the first, ignore the alias.
                    continue
                reader = TagReader(label, port, baud, out_q, stop_evt)
                reader.start()
                claimed_ports[port] = reader
                claimed_labels.add(label)
                print(f"discovered {label} on {port}")
            time.sleep(args.rescan_every)
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        pub_thread.join(timeout=2)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Live range stream, one JSON object per exchange.

  {"tag_label": "T1", "anchor_label": "A1", "range_m": 0.492,
   "rssi": -78.0, "timestamp_sent": 1284551932,
   "timestamp_received": 1789432011.472}

THREE DIFFERENT CLOCKS ARE IN PLAY -- do not mix them up:

  timestamp_sent      The tag's own microsecond counter (the $RNG line's
                      t_local_us field), stamped on the tag as it builds the
                      record, before any buffering. Counts from power-on and
                      wraps every 71.6 min (2^32 us). This is the one to use
                      for update rate and ordering, because it is taken at
                      the source and is immune to USB/OS buffering.

  timestamp_received  This host's wall-clock time when the record was
                      decoded. Useful only for absolute time-of-day: USB-CDC
                      hands over bursts of ~50 records at once, so many
                      records share near-identical values. Do NOT derive
                      rate or ordering from it.

  (the ranging timestamps)
                      Captured in hardware inside the DW1000 at the instant
                      each packet leaves or arrives, in 15.65 ps units.
                      These are what the distance is computed from. They are
                      NOT streamed -- only the resulting tof_ps and range_m
                      are. (The firmware can emit them via out_mask bit 3,
                      but that doubles per-frame output and halves the poll
                      rate, so this tool deliberately does not enable it.)

Usage: raw_json.py [tag ...]      (default: both tags)
"""
import sys, os, time, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb import check_line, BOARDS, popen, sread, get_cfg
import _cleanup

_cleanup.install()

TAGS = sys.argv[1:] or ["ESP_T1", "ESP_T2"]


def build_labels():
    labels = {}
    for name, (port, baud) in BOARDS.items():
        try:
            s = popen(port, baud, settle=0.3)
        except Exception as e:
            print(f"# {name}: {e}", file=sys.stderr)
            continue
        c = get_cfg(s); s.close()
        node, role = c.get("node_id"), c.get("role")
        if not node:
            continue
        if role == "INIT" and c.get("frame_index") is not None:
            labels[node] = f"T{int(c['frame_index']) + 1}"
        elif role == "RESP" and c.get("slot_index") is not None:
            labels[node] = f"A{int(c['slot_index']) + 1}"
    return labels


def main():
    labels = build_labels()

    ports, bufs = {}, {}
    for t in TAGS:
        p, b = BOARDS[t]
        ports[t] = popen(p, b, settle=0.2)
        _cleanup.on_exit(ports[t].close)
        bufs[t] = b''

    while True:
        for t, s in ports.items():
            bufs[t] += sread(s, 4096)
            while b'\n' in bufs[t]:
                raw, bufs[t] = bufs[t].split(b'\n', 1)
                now = time.time()          # stamp at decode, not at print
                body = check_line(raw.decode('ascii', 'replace').strip())
                if body is None or not body.startswith('RNG,'):
                    continue
                q = body.split(',')
                if len(q) < 12:
                    continue
                try:
                    d_mm = int(q[6])
                    flags = int(q[10], 16)
                    if d_mm <= 0 or (flags & 0x04):     # no range / RESP_LOST
                        continue
                    rec = {
                        "tag_label": labels.get(q[3], q[3]),
                        "anchor_label": labels.get(q[4], q[4]),
                        "range_m": round(d_mm / 1000.0, 3),
                        "rssi": float(q[8]),
                        "timestamp_sent": int(q[11]),      # tag's t_local_us
                        "timestamp_received": round(now, 3),
                    }
                except ValueError:
                    continue
                print(json.dumps(rec), flush=True)
        time.sleep(0.02)


main()

#!/usr/bin/env python3
"""Shared helpers for talking to UWB TDMA boards (both platforms)."""
import time, serial

BOARDS = {
    # Roles reassigned 2026-09-15: both ESP32s are now tags (initiators),
    # both DWM1001s are anchors (responders). Keys are named for the label
    # each board reports (T1/T2/A1/A2), derived from frame_index/slot_index,
    # so they stay meaningful if roles move again.
    #
    # Baud: DWM1001 at 115200, ESP32 at 921600. The DWM1001s' J-Link VCOM
    # link showed ~60% line corruption at 921600 even passively (the
    # firmware's own main.c calls 921600 "marginal" for that UART); the
    # ESP32s' CP2104 path is clean at 921600. Baud is per-board and not part
    # of cfg_hash(), so the asymmetry never affects ranging.
    "ESP_T1": ("/dev/cu.usbserial-023BB217",    921600),   # 0x8052 frame 0
    "ESP_T2": ("/dev/cu.usbserial-023BB1E4",    921600),   # 0x8053 frame 1
    "ESP_T3": ("/dev/cu.usbserial-023BB1D1",    921600),   # 0x4051 frame 2
    "ESP_T4": ("/dev/cu.usbserial-023BB185",    921600),   # 0x7851 frame 3
    "DWM_A1": ("/dev/cu.usbmodem0007601315731", 115200),   # 0x0101 slot 0
    "DWM_A2": ("/dev/cu.usbmodem0007601315901", 115200),   # 0x0F70 slot 1
}

# A tag's TDMA position is frame_index (validated < init_count); an anchor's
# is slot_index (validated < resp_count). They are different fields -- setting
# slot_index on a tag does nothing, which is easy to get wrong because every
# board reports both.
#
# Adding a tag costs rate: the superframe is init_count * frame_dur_us, so
# each tag still gets exactly one frame in it. At frame_dur_us=6701 that is
# 74.6 Hz per tag with 2 tags and 49.7 Hz with 3 -- structural, not a fault.
# init_count is hashed, so every node must be moved together or the ones left
# behind have their frames dropped on a cfg_hash mismatch.


def cs(payload):
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return f"\r\n${payload}*{c:02X}\r\n".encode()


def sread(s, n=2048):
    try:
        return s.read(n)
    except serial.SerialException:
        return b''


def popen(port, baud, settle=0.4):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, 0.2
    s.dsrdtr = s.rtscts = False
    s.open()
    time.sleep(settle)
    return s


def lines(s, window=1.2, drop_rng=True):
    t0, pending, out = time.time(), b'', []
    while time.time() - t0 < window:
        chunk = sread(s)
        if not chunk:
            continue
        pending += chunk
        while b'\n' in pending:
            raw, pending = pending.split(b'\n', 1)
            t = raw.decode('ascii', 'replace').strip()
            if '$' not in t:
                continue
            t = t[t.index('$'):]
            if drop_rng and (t.startswith('$RNG') or t.startswith('$ERR,2,cs')):
                continue
            out.append(t)
    return out


def cmd(s, payload, want=('$OK', '$ERR'), attempts=6, window=1.2):
    """Send a command, return the matching reply lines."""
    for _ in range(attempts):
        try:
            s.reset_input_buffer()
            s.write(cs(payload))
            s.flush()
        except serial.SerialException:
            time.sleep(0.2)
            continue
        out = lines(s, window)
        hit = [l for l in out if any(l.startswith(w) for w in want)]
        if hit:
            return hit
        time.sleep(0.15)
    return []


def query(s, payload, want, attempts=8, window=1.8):
    for _ in range(attempts):
        try:
            s.reset_input_buffer()
            s.write(cs(payload))
            s.flush()
        except serial.SerialException:
            time.sleep(0.2)
            continue
        out = lines(s, window)
        if any(w in l for l in out for w in want):
            return out
        time.sleep(0.15)
    return []


def check_line(line):
    """Validate one '$PAYLOAD*XX' line against its XOR checksum.

    Returns the payload (without the leading '$' or the '*XX') or None.

    Serial links drop bytes under load, and a truncated line gets glued to
    the start of the next one -- which previously surfaced as bogus anchor
    IDs like '0$RNG' / '0x0F7$RNG' in the tables, because the torn fragment
    still split into enough comma-separated fields to look plausible. The
    firmware already puts an XOR checksum on every line (output.c's
    out_line), so verifying it rejects every torn or corrupted record
    cleanly instead of letting garbage through.
    """
    if not line or '$' not in line or '*' not in line:
        return None
    line = line[line.rindex('$'):]          # drop anything before the last '$'
    body, _, cks = line[1:].rpartition('*')
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


def get_cfg(s):
    out = query(s, "CFG", ("$C,role",))
    kv = {}
    for l in out:
        if l.startswith("$C,"):
            k, _, v = l.split("*")[0][3:].partition(",")
            kv[k] = v
        elif l.startswith("$CFGBEGIN"):
            for part in l.split("*")[0].split(","):
                if part.startswith("hash="):
                    kv["_hash"] = part[5:]
    return kv


def get_stat(s):
    out = query(s, "STAT", ("$STAT,frames",))
    st = {}
    for l in out:
        if not l.startswith("$STAT,"):
            continue
        parts = l.split("*")[0][6:].split(",")
        for i in range(0, len(parts) - 1, 2):
            st[parts[i]] = parts[i + 1]
    return st


def get_measure(s):
    out = query(s, "MEASURE", ("$MEAS,proc",))
    m = {}
    for l in out:
        p = l.split("*")[0].split(",")
        if len(p) >= 6 and p[0] == "$MEAS":
            m[p[1]] = {"last": p[3], "max": p[5]}
        elif len(p) >= 3 and p[0] == "$MEAS" and p[1] == "tx_late":
            m["tx_late"] = p[2]
    for l in out:
        if l.startswith("$MEAS,tx_late"):
            m["tx_late"] = l.split("*")[0].split(",")[2]
    return m

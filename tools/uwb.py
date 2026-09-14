#!/usr/bin/env python3
"""Shared helpers for talking to UWB TDMA boards (both platforms)."""
import time, serial

BOARDS = {
    "DWM_ANC": ("/dev/cu.usbmodem0007601315731", 115200),
    "DWM_TAG": ("/dev/cu.usbmodem0007601315901", 115200),
    "ESP_ANC": ("/dev/cu.usbserial-023BB1E4",    921600),
    "ESP_TAG": ("/dev/cu.usbserial-023BB217",    921600),
}


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

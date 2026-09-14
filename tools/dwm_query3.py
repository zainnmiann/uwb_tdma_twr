#!/usr/bin/env python3
"""Read-only DWM1001 query, tolerant of J-Link VCOM read glitches."""
import sys, time, serial

PORT = sys.argv[1]
BAUDS = [int(b) for b in sys.argv[2:]] or [115200, 921600]


def cs(payload: str) -> bytes:
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return f"\r\n${payload}*{c:02X}\r\n".encode()


def opened(port, baud):
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0.2
    s.dsrdtr = False
    s.rtscts = False
    s.open()
    return s


def safe_read(s, n=1024):
    try:
        return s.read(n)
    except serial.SerialException:
        return b''


def ask(s, cmd, want, attempts=10, window=1.5):
    best = []
    for _ in range(attempts):
        try:
            s.reset_input_buffer()
            s.write(cs(cmd))
            s.flush()
        except serial.SerialException:
            time.sleep(0.2)
            continue
        t0, pending, out = time.time(), b'', []
        while time.time() - t0 < window:
            chunk = safe_read(s)
            if not chunk:
                continue
            pending += chunk
            while b'\n' in pending:
                raw, pending = pending.split(b'\n', 1)
                line = raw.decode('ascii', 'replace').strip()
                if '$' not in line:
                    continue
                line = line[line.index('$'):]
                if line.startswith('$RNG') or line.startswith('$ERR,2,cs'):
                    continue
                out.append(line)
        if any(w in l for l in out for w in want):
            return out
        if len(out) > len(best):
            best = out
        time.sleep(0.2)
    return best


def main():
    for baud in BAUDS:
        try:
            s = opened(PORT, baud)
        except Exception as e:
            print(f"# open failed at {baud}: {e}")
            continue
        time.sleep(0.4)
        res = ask(s, "MEASURE", ("$MEAS",))
        if not any("$MEAS" in l for l in res):
            print(f"# no $MEAS at {baud} (saw {len(res)} lines)")
            for l in res[:4]:
                print("   ", l)
            s.close()
            continue
        print(f"# responds at {baud} baud\n\n=== $MEASURE ===")
        for l in res:
            print(l)
        for cmd, want in (("STAT", ("$STAT",)), ("CFG", ("$C,role",))):
            print(f"\n=== ${cmd} ===")
            for l in ask(s, cmd, want):
                print(l)
        s.close()
        return
    print("# board did not answer")


main()

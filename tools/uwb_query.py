#!/usr/bin/env python3
"""Read-only query of a UWB TDMA board. Sends only $MEASURE/$STAT/$CFG."""
import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/cu.usbserial-023BB217'
BAUD = 921600


def cs(payload: str) -> str:
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return f"${payload}*{c:02X}\r\n"


def main():
    # dsrdtr/rtscts off so opening does not drive the auto-reset circuit.
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = 0.2
    s.dsrdtr = False
    s.rtscts = False
    s.open()
    try:
        s.dtr = False
        s.rts = False
    except Exception as e:
        print(f"# could not set control lines: {e}")

    time.sleep(0.3)
    s.reset_input_buffer()

    # 1. Passive listen: is it already ranging?
    print("=== passive listen 3s ===")
    t0 = time.time()
    n_rng = 0
    samples = []
    while time.time() - t0 < 3.0:
        line = s.readline().decode('ascii', 'replace').strip()
        if not line:
            continue
        if line.startswith('$RNG'):
            n_rng += 1
            if len(samples) < 5:
                samples.append(line)
        elif len(samples) < 5:
            samples.append(line)
    for x in samples:
        print(x)
    print(f"# $RNG lines in 3s: {n_rng}  ({n_rng/3.0:.1f}/s)")

    # 2. Read-only queries.
    for cmd in ("MEASURE", "STAT", "CFG"):
        print(f"\n=== ${cmd} ===")
        s.reset_input_buffer()
        s.write(cs(cmd).encode())
        s.flush()
        t0 = time.time()
        got = 0
        while time.time() - t0 < 2.5:
            line = s.readline().decode('ascii', 'replace').strip()
            if not line:
                continue
            if line.startswith('$RNG'):
                continue          # suppress the ranging stream
            print(line)
            got += 1
            if got > 120:
                break
    s.close()


main()

#!/usr/bin/env python3
"""READ-ONLY restore point: dump $CFG + $STAT + $MEASURE from every board.

Writes one <label>.cfg file per board containing the exact $SET lines needed
to restore it, plus a combined transcript. Sends no command that changes
state (only $CFG / $STAT / $MEASURE).
"""
import json, os, sys, time, serial

OUT = os.path.dirname(os.path.abspath(__file__))

PORTS = [
    ("/dev/cu.usbmodem0007601315731", [115200, 921600]),   # DWM1001 ANC1
    ("/dev/cu.usbmodem0007601315901", [115200, 921600]),   # DWM1001 TAG1
    ("/dev/cu.usbserial-023BB1E4",    [921600, 115200]),   # ESP32
    ("/dev/cu.usbserial-023BB217",    [921600, 115200]),   # ESP32
]


def cs(payload):
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return f"\r\n${payload}*{c:02X}\r\n".encode()


def safe_read(s, n=2048):
    try:
        return s.read(n)
    except serial.SerialException:
        return b''


def ask(s, cmd, want, attempts=8, window=1.8):
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
        time.sleep(0.15)
    return best


def open_at(port, baud):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, 0.2
    s.dsrdtr = s.rtscts = False
    s.open()
    time.sleep(0.35)
    return s


def main():
    summary = {}
    for port, bauds in PORTS:
        print(f"\n{'=' * 70}\n{port}")
        got = None
        for baud in bauds:
            try:
                s = open_at(port, baud)
            except Exception as e:
                print(f"  open failed at {baud}: {e}")
                continue
            cfg = ask(s, "CFG", ("$C,role",))
            if any("$C,role" in l for l in cfg):
                got = (s, baud, cfg)
                break
            s.close()
        if not got:
            print("  !! no response")
            continue
        s, baud, cfg = got
        kv = {}
        for l in cfg:
            if l.startswith("$C,"):
                body = l.split("*")[0][3:]
                k, _, v = body.partition(",")
                kv[k] = v
        label = f"{kv.get('role','?')}_{kv.get('node_id','?')}"
        print(f"  baud={baud}  role={kv.get('role')}  node={kv.get('node_id')} "
              f"hash={[l for l in cfg if 'CFGBEGIN' in l]}")

        meas = ask(s, "MEASURE", ("$MEAS",))
        stat = ask(s, "STAT", ("$STAT",))
        s.close()

        summary[port] = {"baud": baud, "label": label, "cfg": kv,
                         "measure": meas, "stat": stat}

        # Restore script: only keys that are settable.
        skip = {"node_id"}
        with open(os.path.join(OUT, f"restore_{label}.txt"), "w") as f:
            f.write(f"# restore for {port} ({label}) baud={baud}\n")
            f.write("$RANGE,STOP\n")
            for k, v in kv.items():
                if k in skip or v == "":
                    continue
                f.write(f"$SET,{k},{v}\n")
            f.write("$SAVE\n$RESET\n")
        print(f"  -> restore_{label}.txt   ({len(kv)} keys)")
        for l in meas:
            print("   ", l)

    with open(os.path.join(OUT, "baseline_all.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print(f"\nwrote baseline_all.json ({len(summary)} boards)")


main()

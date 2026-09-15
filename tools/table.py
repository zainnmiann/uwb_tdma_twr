#!/usr/bin/env python3
"""Live per-tag / per-anchor range table.

Rows are tag->anchor pairs; each shows the latest range (m), rolling
standard deviation (cm), update rate (Hz) and cumulative sample count.

    tag   anchor      range        sd       rate        n
    ---------------------------------------------------------
    T1    A1         0.492m     8.2cm    54.2 Hz     2714
    T1    A2         0.399m     7.9cm    52.6 Hz     2633
    T2    A1         0.446m     1.4cm    54.6 Hz     2735
    T2    A2         0.535m     2.9cm    53.4 Hz     2671

Labels (T1/T2, A1/A2) are read live from each board's frame_index /
slot_index at startup, so they track whatever is actually provisioned.

Usage: table.py [stale_seconds]      (default 3.0)
       table.py --window N           rolling window for sd (default 50)
"""
import sys, os, time, statistics
from collections import deque, defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb import check_line, BOARDS, popen, sread, get_cfg
import _cleanup

_cleanup.install()

argv = sys.argv[1:]
WIN = 50
if "--window" in argv:
    i = argv.index("--window")
    WIN = int(argv[i + 1]); del argv[i:i + 2]
STALE = float(argv[0]) if argv else 3.0
TAGS = ["ESP_T1", "ESP_T2"]


def build_labels():
    labels = {}
    for name, (port, baud) in BOARDS.items():
        try:
            s = popen(port, baud, settle=0.3)
        except Exception:
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
    cell = defaultdict(lambda: {"d": None, "t": 0.0, "hist": deque(maxlen=WIN),
                                "n": 0, "t0": None})

    ports, bufs = {}, {}
    for t in TAGS:
        p, b = BOARDS[t]
        ports[t] = popen(p, b, settle=0.2)
        _cleanup.on_exit(ports[t].close); bufs[t] = b''

    last_draw = 0.0
    try:
        while True:
            now = time.time()
            for t, s in ports.items():
                bufs[t] += sread(s, 4096)
                while b'\n' in bufs[t]:
                    raw, bufs[t] = bufs[t].split(b'\n', 1)
                    l = raw.decode('ascii', 'replace').strip()
                    body = check_line(l)
                    if body is None or not body.startswith('RNG,'):
                        continue
                    q = body.split(',')
                    if len(q) < 12:
                        continue
                    q = ['$' + q[0]] + q[1:]
                    try:
                        tag_l = labels.get(q[3], q[3])
                        anc_l = labels.get(q[4], q[4])
                        d_mm = int(q[6]); flags = int(q[10], 16)
                    except ValueError:
                        continue
                    if d_mm <= 0 or (flags & 0x04):
                        continue
                    c = cell[(tag_l, anc_l)]
                    if c["t0"] is None:
                        c["t0"] = now
                    c["d"] = d_mm; c["t"] = now
                    c["n"] += 1; c["hist"].append(d_mm)

            if now - last_draw >= 0.5:
                last_draw = now
                sys.stdout.write('\033[H\033[J')
                # Only rows that are actually live -- a tag/anchor pair that
                # has gone quiet past STALE is dropped entirely rather than
                # shown as '--', so the table lists exactly what is present.
                live = [(tg, an) for (tg, an) in sorted(cell)
                        if cell[(tg, an)]["d"] is not None
                        and (now - cell[(tg, an)]["t"]) <= STALE]
                print(f"{'tag':<6}{'anchor':<9}{'range':>9}{'sd':>9}"
                      f"{'rate':>11}{'n':>9}")
                print("-" * 53)
                for (tg, an) in live:
                    c = cell[(tg, an)]
                    rng = f"{c['d']/1000:.3f}m"
                    sd = (f"{statistics.pstdev(c['hist'])/10:.1f}cm"
                          if len(c["hist"]) > 1 else "--")
                    el = max(now - (c["t0"] or now), 1e-3)
                    rate = f"{c['n']/el:.1f} Hz"
                    print(f"{tg:<6}{an:<9}{rng:>9}{sd:>9}{rate:>11}{c['n']:>9}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        for s in ports.values():
            s.close()


main()

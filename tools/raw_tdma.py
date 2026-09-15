#!/usr/bin/env python3
"""Raw TDMA trace: one line per $RNG record, in schedule order
(superframe -> frame_idx=tag's frame slot -> slot=anchor's response slot),
exactly as the firmware transmits it -- no averaging, no aggregation.

Tags/anchors are shown as T<n>/A<n> (position in the schedule, 1-based)
instead of their raw node_id, queried live from each board's $CFG at
startup -- so labels track whatever is actually configured right now.

Usage: raw_tdma.py [tag ...]   (default: both tags)
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb import check_line, BOARDS, popen, sread, get_cfg
import _cleanup

_cleanup.install()

TAGS = sys.argv[1:] or ["ESP_T1", "ESP_T2"]
FL = [(0x01,"SS_FB"),(0x02,"NLOS"),(0x04,"RESP_LOST"),(0x08,"REP_LOST"),
      (0x10,"DESYNC"),(0x20,"HASH_MM")]


def build_labels():
    """node_id (as '0xXXXX') -> 'T<frame_index+1>' / 'A<slot_index+1>',
    queried from every board's own $CFG so labels always match reality."""
    labels = {}
    for name, (port, baud) in BOARDS.items():
        try:
            s = popen(port, baud, settle=0.3)
        except Exception as e:
            print(f"# {name}: could not open to read labels: {e}")
            continue
        cfg = get_cfg(s)
        s.close()
        node = cfg.get("node_id")
        role = cfg.get("role")
        if not node:
            continue
        if role == "INIT":
            idx = cfg.get("frame_index")
            if idx is not None:
                labels[node] = f"T{int(idx) + 1}"
        elif role == "RESP":
            idx = cfg.get("slot_index")
            if idx is not None:
                labels[node] = f"A{int(idx) + 1}"
    return labels


def lbl(labels, addr):
    return labels.get(addr, addr)


def main():
    labels = build_labels()
    print("# labels:", ", ".join(f"{v}={k}" for k, v in sorted(labels.items(), key=lambda kv: kv[1])))

    ports, bufs = {}, {}
    for t in TAGS:
        p, b = BOARDS[t]
        try:
            ports[t] = popen(p, b, settle=0.2)
            _cleanup.on_exit(ports[t].close)
            bufs[t] = b''
        except Exception as e:
            print(f"# {t}: {e}")

    hdr = f"{'src':9} {'sfn':>6} {'frame':>5} {'tag':>5} {'anchor':>7} {'slot':>4} {'range':>9} {'tof_ps':>10} {'rxp':>7} {'fpp':>7} {'flags'}"
    print(hdr)
    print("-" * len(hdr))

    try:
        while True:
            for t, s in ports.items():
                bufs[t] += sread(s, 4096)
                while b'\n' in bufs[t]:
                    raw, bufs[t] = bufs[t].split(b'\n', 1)
                    l = raw.decode('ascii', 'replace').strip()
                    body = check_line(l)
                    if body is None or not body.startswith('RNG,'):
                        continue
                    p = body.split(',')
                    p = ['$' + p[0]] + p[1:]
                    if len(p) < 12:
                        continue
                    try:
                        sfn, frame, tag, anchor, slot = p[1], p[2], p[3], p[4], int(p[5])
                        d_mm, tof_ps = int(p[6]), int(p[7])
                        rxp, fpp = p[8], p[9]
                        flags = int(p[10], 16)
                    except ValueError:
                        continue
                    fl = ",".join(n for b, n in FL if flags & b) or "-"
                    rng = f"{d_mm/1000:.3f}m" if d_mm > 0 else "--"
                    print(f"{t:9} {sfn:>6} {frame:>5} {lbl(labels, tag):>5} "
                          f"{lbl(labels, anchor):>7} {slot:>4} {rng:>9} "
                          f"{tof_ps:>10} {rxp:>7} {fpp:>7} {fl}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        for s in ports.values():
            s.close()


main()

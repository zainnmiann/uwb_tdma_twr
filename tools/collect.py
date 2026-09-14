#!/usr/bin/env python3
"""Listen on both tags, parse $RNG, and report per-anchor rate/range/flags.

Usage: collect.py [seconds]
"""
import sys, os, time, statistics, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb import BOARDS, popen, sread, get_stat, get_measure

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
TAGS = ["DWM_TAG", "ESP_TAG"]

FLAGBITS = [(0x01, "SS_FB"), (0x02, "NLOS"), (0x04, "RESP_LOST"),
            (0x08, "REP_LOST"), (0x10, "DESYNC"), (0x20, "HASH_MM")]


def parse(line):
    p = line.split("*")[0].split(",")
    if len(p) < 12 or p[0] != "$RNG":
        return None
    try:
        return {"tag": p[3], "anchor": p[4], "slot": int(p[5]),
                "d_mm": int(p[6]), "rxp": p[8], "flags": int(p[10], 16)}
    except ValueError:
        return None


def main():
    ports = {}
    for t in TAGS:
        port, baud = BOARDS[t]
        try:
            ports[t] = popen(port, baud, settle=0.2)
        except Exception as e:
            print(f"{t}: open failed: {e}")
    for s in ports.values():
        s.reset_input_buffer()

    recs = {t: [] for t in ports}
    bufs = {t: b'' for t in ports}
    t0 = time.time()
    while time.time() - t0 < SECS:
        for t, s in ports.items():
            chunk = sread(s, 4096)
            if not chunk:
                continue
            bufs[t] += chunk
            while b'\n' in bufs[t]:
                raw, bufs[t] = bufs[t].split(b'\n', 1)
                line = raw.decode('ascii', 'replace').strip()
                if '$RNG' not in line:
                    continue
                r = parse(line[line.index('$RNG'):])
                if r:
                    recs[t].append(r)

    print(f"=== {SECS:.0f}s capture ===")
    out = {}
    for t, rs in recs.items():
        print(f"\n--- {t}  ({len(rs)} records, {len(rs)/SECS:.1f}/s) ---")
        byslot = {}
        for r in rs:
            byslot.setdefault(r["slot"], []).append(r)
        for slot in sorted(byslot):
            g = byslot[slot]
            good = [x for x in g if x["flags"] & 0x04 == 0 and x["d_mm"] > 0]
            anchors = set(x["anchor"] for x in g if x["anchor"] != "0x0000")
            fl = {}
            for x in g:
                for bit, nm in FLAGBITS:
                    if x["flags"] & bit:
                        fl[nm] = fl.get(nm, 0) + 1
            d = [x["d_mm"] for x in good]
            ds = (f"mean={statistics.mean(d):7.0f}mm "
                  f"sd={statistics.pstdev(d):5.0f}" if len(d) > 1 else
                  (f"d={d[0]}mm" if d else "no valid range"))
            print(f"  slot {slot}: {len(g):4d} rec  {len(good):4d} good "
                  f"({len(good)/SECS:5.1f}/s)  anchors={sorted(anchors) or '-'}")
            print(f"           {ds}   flags={fl or '{}'}")
            out[f"{t}_slot{slot}"] = {"rec": len(g), "good": len(good),
                                      "rate": len(good)/SECS,
                                      "anchors": sorted(anchors),
                                      "mean_mm": statistics.mean(d) if d else None,
                                      "flags": fl}
        for s in [ports[t]]:
            st = get_stat(s)
            m = get_measure(s)
            print(f"  STAT: tx_late={st.get('tx_late')} ss_fb={st.get('ss_fallback')} "
                  f"ranges={st.get('ranges')} hash_mm={st.get('hash_mismatch')} "
                  f"sync={st.get('sync')} rx_err={st.get('rx_errors')}")
            print(f"  MEAS: {m}")
            out[t + "_stat"] = st
            out[t + "_meas"] = m
    for s in ports.values():
        s.close()
    return out


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Live ranging table. Usage: live.py [tag ...]   (default: both tags)"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb import BOARDS, popen, sread

TAGS = sys.argv[1:] or ["DWM_TAG", "ESP_TAG"]
FL = [(0x01,"SS"),(0x02,"NLOS"),(0x04,"RLOST"),(0x08,"PLOST"),(0x10,"DESYNC"),(0x20,"HASH")]
st, ports, bufs = {}, {}, {}

for t in TAGS:
    p, b = BOARDS[t]
    try:
        ports[t] = popen(p, b, settle=0.2); bufs[t] = b''
    except Exception as e:
        print(f"{t}: {e}")

last = 0
try:
    while True:
        for t, s in ports.items():
            bufs[t] += sread(s, 4096)
            while b'\n' in bufs[t]:
                raw, bufs[t] = bufs[t].split(b'\n', 1)
                l = raw.decode('ascii', 'replace').strip()
                if '$RNG' not in l:
                    continue
                p = l[l.index('$RNG'):].split('*')[0].split(',')
                if len(p) < 12:
                    continue
                try:
                    k = (t, int(p[5]))
                    d = st.setdefault(k, {"n":0,"ok":0,"mm":0,"anchor":"","rxp":"","f":0,"t0":time.time()})
                    d["n"] += 1; d["anchor"] = p[4]; d["f"] = int(p[10],16); d["rxp"] = p[8]
                    if not (d["f"] & 0x04) and int(p[6]) > 0:
                        d["ok"] += 1; d["mm"] = int(p[6])
                except ValueError:
                    pass
        if time.time() - last > 0.5:
            last = time.time()
            os.system('clear')
            print(f"{'tag':9} {'slot':>4} {'anchor':>8} {'range':>9} {'rxp':>7} {'Hz':>6} {'ok%':>5}  flags")
            print("-" * 68)
            for (t, slot), d in sorted(st.items()):
                el = max(time.time() - d["t0"], 1e-3)
                fl = " ".join(n for b, n in FL if d["f"] & b)
                print(f"{t:9} {slot:>4} {d['anchor']:>8} {d['mm']/1000:>8.3f}m "
                      f"{d['rxp']:>7} {d['ok']/el:>6.1f} {100*d['ok']//max(d['n'],1):>4}%  {fl}")
            print("\nCtrl-C to stop")
        time.sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    for s in ports.values():
        s.close()

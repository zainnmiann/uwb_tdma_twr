"""Per tag / per anchor link quality from the live $RNG stream.

    python3 tools/quality.py [seconds]

Reads both tags concurrently for the given window and reports, per pair:
rate (Hz), sample count, the share of clean DS-TWR fixes (good%), mean range,
and standard deviation both raw and outlier-robust.

Two details that make the numbers trustworthy:

  - A $RNG line with anchor 0x0000 and flag RNG_FLAG_RESP_LOST (0x04) is the
    tag's MISS placeholder, not a real anchor. Counting it as a third anchor
    invents a phantom with range 0.000; it is tallied as a loss instead.

  - sd_cm vs sdR_cm: RNG_FLAG_SS_FALLBACK measurements are clock-offset
    sensitive and emit occasional wild values (40 m has been seen), which
    dominate a naive standard deviation. sdR_cm excludes samples more than
    0.5 m from the median, so a gap between the two columns means SS-TWR
    fallback is firing -- look at the flags column, not just the SD.

Every line is checksum-validated; torn lines are counted, not parsed.
"""
import sys, time, threading, collections, statistics
sys.path.insert(0, __import__('os').path.dirname(__file__) or '.')
from uwb import BOARDS, popen, check_line
DUR = float(sys.argv[1]) if len(sys.argv)>1 else 40.0
S={n:popen(p,b,settle=0.6) for n,(p,b) in BOARDS.items() if n.startswith('ESP')}
LBL={'ESP_T1':'T1','ESP_T2':'T2'}
rng=collections.defaultdict(list); fl=collections.defaultdict(collections.Counter)
tot=collections.Counter(); sf=collections.defaultdict(set); stop=[False]
def rd(n,s):
    while not stop[0]:
        try: ln=s.readline().decode('ascii','replace').strip()
        except: break
        if not ln.startswith('$RNG') or not check_line(ln): continue
        f=ln.split('*')[0].split(',')
        if len(f)!=12: continue
        try:
            fg=int(f[10],16); ridx=int(f[5]); t=LBL[n]
        except: continue
        sf[t].add(f[1]); k=(t,ridx); tot[k]+=1
        if fg & 0x04: fl[k]['RESP_LOST']+=1; continue
        if fg & 0x08: fl[k]['REPORT_LOST']+=1
        if fg & 0x01: fl[k]['SS_FB']+=1
        if fg & 0x02: fl[k]['NLOS']+=1
        if fg & 0x10: fl[k]['DESYNC']+=1
        if fg==0: fl[k]['OK']+=1
        try: rng[k].append(float(f[6])/1000.0)
        except: pass
[threading.Thread(target=rd,args=(n,s),daemon=True).start() for n,s in S.items()]
t0=time.time(); time.sleep(DUR); el=time.time()-t0; stop[0]=True; time.sleep(0.4)
print(f"{'tag':<4}{'anch':<6}{'Hz':>7}{'n':>7}{'good%':>8}{'range_m':>9}{'sd_cm':>8}{'sdR_cm':>8}   flags")
for k in sorted(tot):
    v=rng[k]; N=tot[k]; c=fl[k]
    med=statistics.median(v) if v else 0
    rob=[x for x in v if abs(x-med)<0.5]
    sd=statistics.pstdev(v)*100 if len(v)>1 else 0
    sdr=statistics.pstdev(rob)*100 if len(rob)>1 else 0
    mean=statistics.mean(rob) if rob else 0
    print(f"{k[0]:<4}A{k[1]+1:<5}{N/el:>7.1f}{N:>7}{100.0*c['OK']/max(1,N):>7.1f}%{mean:>9.3f}{sd:>8.1f}{sdr:>8.2f}   {dict(c)}")
for t in sorted(sf): print(f"  {t}: superframes={len(sf[t])} poll={len(sf[t])/el:.1f}Hz")
for s in S.values(): s.close()

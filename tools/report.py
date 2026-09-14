#!/usr/bin/env python3
"""Format the pitch sweep results."""
import json, sys, glob, os
HERE = os.path.dirname(os.path.abspath(__file__))
f = sys.argv[1] if len(sys.argv) > 1 else sorted(
    glob.glob(os.path.join(HERE, "..", "measurements", "pitch_sweep_*.json")))[-1]
d = json.load(open(f))

print(f"{os.path.basename(f)}\n")
hdr = (f"{'pitch':>6} {'hash':>7} | {'DWM tag':^28} | {'ESP tag':^28}")
print(hdr); print(f"{'':>6} {'':>7} | {'late':>5} {'ss_fb':>6} {'rng':>6} {'sync':>7} | "
                 f"{'late':>5} {'ss_fb':>6} {'rng':>6} {'sync':>7}")
print("-" * len(hdr))
for pitch in sorted(d, key=lambda x: -int(x)):
    v = d[pitch]
    if "error" in v:
        print(f"{pitch:>6} {'--':>7} | {v['error']}")
        continue
    c = v.get("capture", {})
    row = f"{pitch:>6} {v.get('hash','?'):>7} |"
    for t in ("DWM_TAG", "ESP_TAG"):
        s = c.get(t + "_stat", {})
        late = s.get("tx_late", "?"); ss = s.get("ss_fallback", "?")
        rng = s.get("ranges", "?"); sy = (s.get("sync") or "-")[:7]
        row += f" {late:>5} {ss:>6} {rng:>6} {sy:>7} |"
    print(row)

print("\nper-anchor detail (good ranges/s, mean mm):")
for pitch in sorted(d, key=lambda x: -int(x)):
    c = d[pitch].get("capture", {})
    bits = []
    for k, v in sorted(c.items()):
        if not k.endswith(tuple(f"slot{i}" for i in range(4))):
            continue
        mm = v.get("mean_mm")
        bits.append(f"{k}={v['rate']:.1f}/s"
                    + (f"@{mm:.0f}mm" if mm else "@--")
                    + (f" {list(v['flags'])}" if v.get("flags") else ""))
    if bits:
        print(f"  {pitch:>6}: " + "  ".join(bits))

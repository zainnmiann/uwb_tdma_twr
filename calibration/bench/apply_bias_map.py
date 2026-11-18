#!/usr/bin/env python3
"""Remove DW1000 range bias with a calibration map (APS011 approach).

The bench runs in summary.csv give, per distance, the MEAN MEASURED range and
its bias (measured - true). Runs at the same distance are pooled (weighted by
reading count). Between points the bias is interpolated with straight
segments; outside the table it is held at the end value.

APS011: bias is ~0 at the distance where the antenna delay was calibrated,
negative closer in, positive further out. So the map is re-zeroed for each
link at the distances where ITS two boards were calibrated. On 21-22 Sep 2026
every board was calibrated against T1 (antenna delays in
~/Documents/uwb_tdma_twr/measurements/ant_delay_*.json):

    pair (T1, X) zeroed at CAL[X]   ->  d_T1 + d_X = -map(CAL[X])

so for a tag t and anchor a, with T1's own share taken as 0 (T1/T4 split):

    link (T1, a):  bias(r) = map(r) - map(CAL[a])
    link (t,  a):  bias(r) = map(r) - map(CAL[a]) - map(CAL[t])

and each reading is corrected with the bias at its own measured range:

    r_corrected = r_measured - bias(r_measured)

The bench map was measured DWM1001 <-> DWM1001; using it for ESP32 boards
(T1-T4, A7) assumes the same DW1000 bias curve.

Usage:
    python3 apply_bias_map.py <UWB-Results dir> <out dir> [--survey xlsx]
           [--zero-at M]        # instead: one global zero for every link
"""
import argparse, csv, glob, json, math, os, sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
# Calibration distance (m) of each board against T1, 21-22 Sep 2026.
CAL = {"T2": 1.502, "T3": 1.630, "T4": 1.630, "T5": 1.502, "T6": 1.600,
       "A1": 5.480, "A2": 5.480, "A3": 5.480, "A4": 5.480, "A5": 5.480,
       "A6": 5.480, "A7": 6.585, "A8": 5.252}
HW = {"T1": "ESP32", "T2": "ESP32", "T3": "ESP32", "T4": "ESP32", "T5": "DWM", "T6": "DWM",
      "A1": "DWM", "A2": "DWM", "A3": "DWM", "A4": "DWM", "A5": "DWM", "A6": "DWM",
      "A7": "ESP32", "A8": "DWM"}


def load_map(summary):
    pool = defaultdict(lambda: [0, 0.0, 0.0])          # true_m -> [n, sum measured, sum bias]
    for r in csv.DictReader(open(summary)):
        n = int(r["n"])
        p = pool[float(r["true_m"])]
        p[0] += n
        p[1] += n * float(r["mean_m"])
        p[2] += n * float(r["bias_cm"]) / 100.0
    tab = sorted((s / n, b / n, t) for t, (n, s, b) in pool.items())
    return [x for x, _, _ in tab], [y for _, y, _ in tab], [t for _, _, t in tab]


def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("out")
    ap.add_argument("--survey", help="survey workbook, adds true range / error columns")
    ap.add_argument("--zero-at", type=float, default=None,
                    help="one global zero distance (m) for every link instead of per-board")
    ap.add_argument("--summary", default=os.path.join(HERE, "summary.csv"))
    a = ap.parse_args()

    xs, ys, ts = load_map(a.summary)
    m = lambda d: interp(d, xs, ys)
    print("calibration map (mean measured -> bias):")
    for x, y, t in zip(xs, ys, ts):
        print(f"  {t:6.3f} m true  {x:7.3f} m measured  bias {y * 100:+6.1f} cm")

    def offset(tag, anc):
        if a.zero_at is not None:
            return m(a.zero_at)
        return m(CAL[anc]) + (m(CAL[tag]) if tag != "T1" else 0.0)

    if a.survey:
        sys.path.insert(0, os.path.join(HERE, ".."))
        from correct_dwm_ranges import survey_truth
    os.makedirs(a.out, exist_ok=True)
    rows, offs = [], {}
    for d in sorted(glob.glob(os.path.join(a.src, "position*"))):
        pos = os.path.basename(d).split("-")[0].replace("position", "")
        if a.survey:
            meta = json.load(open(os.path.join(d, "metadata.json")))
            anc_xyz, tag_xyz = survey_truth(a.survey, meta["anchors"])
        for line in open(os.path.join(d, "raw.jsonl")):
            r = json.loads(line)
            p = json.loads(r["line"].split(" ", 1)[1])
            tg, an = p["tag_label"], p["anchor_label"]
            if tg not in HW or an not in CAL:
                continue
            o = offs.setdefault((tg, an), offset(tg, an))
            rm = p["range_m"]
            bias = m(rm) - o
            row = {"position": pos, "received_at": r["received_at"], "tag": tg, "tag_hw": HW[tg],
                   "anchor": an, "anchor_hw": HW[an], "rssi_dbm": p["rssi"], "range_raw_m": rm,
                   "bias_applied_m": round(bias, 4), "range_corrected_m": round(rm - bias, 4)}
            if a.survey:
                tr = math.dist(tag_xyz[f"{pos}{tg}"], anc_xyz[an]) if an in anc_xyz else None
                row["true_range_m"] = round(tr, 4) if tr is not None else ""
                row["err_raw_cm"] = round((rm - tr) * 100, 2) if tr is not None else ""
                row["err_corrected_cm"] = round((rm - bias - tr) * 100, 2) if tr is not None else ""
            rows.append(row)
    print("per-link zero offsets subtracted from the map (cm):")
    for tg in sorted({k[0] for k in offs}):
        print("  " + tg + ": " + "  ".join(f"{an} {offs[(tg, an)] * 100:+.1f}"
                                          for an in sorted(x for t, x in offs if t == tg)))
    name = "ranges_bias_map" + (f"_zero{a.zero_at:g}m" if a.zero_at is not None else "_all_tags") + ".csv"
    with open(os.path.join(a.out, name), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {len(rows)} readings -> {os.path.join(a.out, name)}")


if __name__ == "__main__":
    main()

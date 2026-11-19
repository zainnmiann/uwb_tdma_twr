#!/usr/bin/env python3
"""Fix the recorded ranges of the 24 Sep 2026 survey runs (all tags T1-T6).

Three steps, applied per tag -> anchor link:

1. First-path filter. Multipath and a weak direct path make a DW1000 range
   read LONG, so each reading is replaced by the 20th percentile of the 9
   readings centred on it (~2 s) instead of the median.

2. Calibration map (DW1000 APS011). Per tag, the filtered readings on the
   surveyed anchors A1-A5 are binned by measured distance (4 m bins); each bin
   gives (mean measured distance, mean bias). The bias at each reading's own
   measured distance is subtracted (linear between points, ends held).

3. Geometric consistency. Each tag was static during a run and ranged up to
   seven anchors, so its ranges must meet at one point. The tag position is
   solved in 3D from the link means (robust Huber least squares, 10 cm) using
   ONLY the anchor coordinates, and each link's mean is moved onto the
   distance from that point; the reading-to-reading variation is kept. This
   removes most of the per-link (multipath) bias that steps 1-2 cannot.
   --fixed-z-from-survey solves with the tag height taken from the survey
   instead of 3D (better, but uses survey information).

Caveat: after step 3 the ranges of one tag are no longer independent: they
are consistent with the solved position by construction. Do not use them to
judge positioning accuracy; use range_mapped_m for that.

Truth for scoring: surveyed tag points moved into the metadata frame, metadata
anchor coordinates (A1-A7). The script prints the result and a
leave-one-position-out check (maps rebuilt from 4 positions, applied to the
5th; step 3 uses no truth).

Usage:
    python3 fix_recorded_ranges.py <UWB-Results dir> <survey.xlsx> <out dir> [--fixed-z-from-survey]
Needs numpy and openpyxl.
"""
import csv, glob, json, math, os, sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "bench"))
from correct_dwm_ranges import survey_truth          # noqa: E402
from apply_bias_map import HW                         # noqa: E402

TAGS = ["T1", "T2", "T3", "T4", "T5", "T6"]
ANCHORS = ["A1", "A2", "A3", "A4", "A5", "A6", "A7"]
FIT_ANCHORS = {"A1", "A2", "A3", "A4", "A5"}
WINDOW, QUANTILE = 9, 0.20
BIN_EDGES = [4.0, 8.0, 12.0, 16.0, 20.0]
MIN_BIN = 50
HUBER_M = 0.10
MIN_LINK = 30           # readings a link needs to take part in the position solve


def first_path(x):
    h = WINDOW // 2
    return np.array([np.quantile(x[max(0, i - h):i + h + 1], QUANTILE) for i in range(len(x))])


def build_maps(fp, truth, exclude_pos=None):
    acc = defaultdict(lambda: defaultdict(list))
    for (pos, tg, an), x in fp.items():
        tr = truth.get((pos, tg, an))
        if an not in FIT_ANCHORS or tr is None or pos == exclude_pos:
            continue
        for i in range(len(BIN_EDGES) - 1):
            s = (x >= BIN_EDGES[i]) & (x < BIN_EDGES[i + 1])
            if s.any():
                acc[tg][i].append((x[s], x[s] - tr))
    maps = {}
    for tg, bins in acc.items():
        pts = []
        for i, parts in bins.items():
            m = np.concatenate([p[0] for p in parts]); e = np.concatenate([p[1] for p in parts])
            if len(m) >= MIN_BIN:
                pts.append((m.mean(), e.mean(), len(m), BIN_EDGES[i], BIN_EDGES[i + 1]))
        maps[tg] = sorted(pts)
    return maps


def apply_map(tbl, x):
    return x - np.interp(x, [p[0] for p in tbl], [p[1] for p in tbl])


def solve(anc, rng, z=None):
    p = np.r_[anc[:, :2].mean(0), z if z is not None else 1.0]
    w = np.ones(len(rng))
    for it in range(60):
        d = p - anc
        n = np.linalg.norm(d, axis=1)
        res = n - rng
        J = d / n[:, None]
        if z is not None:
            J = J[:, :2]
        sw = np.sqrt(w)
        dp, *_ = np.linalg.lstsq(J * sw[:, None], -res * sw, rcond=None)
        if z is not None:
            p[:2] += dp
        else:
            p += dp
        a = np.abs(res)
        w = np.where(a <= HUBER_M, 1.0, HUBER_M / np.maximum(a, 1e-9))
        if it > 10 and np.abs(dp).max() < 1e-6:
            break
    return p


def pipeline(fp, truth, anc_xyz, tag_z, maps):
    """Steps 2-3. Returns mapped and final range arrays per link, and solved positions."""
    mapped = {k: apply_map(maps[k[1]], x) for k, x in fp.items()}
    final = dict(mapped)
    solved = {}
    for pos in sorted({k[0] for k in fp}):
        for tg in TAGS:
            ks = [k for k in mapped if k[0] == pos and k[1] == tg and k[2] in anc_xyz and len(mapped[k]) >= MIN_LINK]
            if len(ks) < 4:
                continue
            A = np.array([anc_xyz[k[2]] for k in ks])
            p = solve(A, np.array([mapped[k].mean() for k in ks]), tag_z.get((pos, tg)))
            solved[(pos, tg)] = (p, [k[2] for k in ks])
            for k in ks:
                final[k] = mapped[k] - mapped[k].mean() + np.linalg.norm(p - np.array(anc_xyz[k[2]]))
    return mapped, final, solved


def score(arrs, truth, sel):
    e = np.concatenate([(v - truth[k]) * 100 for k, v in arrs.items() if truth.get(k) is not None and sel(k)])
    return e.mean(), math.sqrt((e ** 2).mean()), 100 * np.mean(np.abs(e) <= 10)


def main():
    args = sys.argv[1:]
    fixz = "--fixed-z-from-survey" in args
    args = [a for a in args if a != "--fixed-z-from-survey"]
    if len(args) != 3:
        sys.exit(__doc__)
    src, xlsx, out = args
    os.makedirs(out, exist_ok=True)

    recs = defaultdict(list)
    truth, anc_xyz, tag_z = {}, {}, {}
    for d in sorted(glob.glob(os.path.join(src, "position*"))):
        pos = os.path.basename(d).split("-")[0].replace("position", "")
        meta = json.load(open(os.path.join(d, "metadata.json")))["anchors"]
        _, tag_xyz = survey_truth(xlsx, meta)
        anc_xyz = {f"A{k[1:].lstrip('0')}": tuple(v["xyz"]) for k, v in meta.items()}
        for line in open(os.path.join(d, "raw.jsonl")):
            r = json.loads(line)
            p = json.loads(r["line"].split(" ", 1)[1])
            tg, an = p["tag_label"], p["anchor_label"]
            if tg not in TAGS or an not in ANCHORS:
                continue
            recs[(pos, tg, an)].append((r["received_at"], p["rssi"], p["range_m"]))
            truth[(pos, tg, an)] = math.dist(tag_xyz[f"{pos}{tg}"], anc_xyz[an]) if an in anc_xyz else None
            if fixz:
                tag_z[(pos, tg)] = tag_xyz[f"{pos}{tg}"][2]

    raw = {k: np.array([x for _, _, x in v]) for k, v in recs.items()}
    fp = {k: first_path(x) for k, x in raw.items()}
    maps = build_maps(fp, truth)
    mapped, final, solved = pipeline(fp, truth, anc_xyz, tag_z, maps)

    # ---- files
    with open(os.path.join(out, "calibration_map_firstpath.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["tag", "tag_hw", "bin_lo_m", "bin_hi_m", "n_readings", "mean_measured_m", "bias_cm"])
        for tg in TAGS:
            for m, b, n, lo, hi in maps[tg]:
                w.writerow([tg, HW[tg], lo, hi, n, round(m, 4), round(b * 100, 2)])
    with open(os.path.join(out, "tag_positions_solved.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["position", "tag", "x_m", "y_m", "z_m", "anchors_used", "z_source"])
        for (pos, tg), (p, used) in sorted(solved.items()):
            w.writerow([pos, tg, *(round(float(v), 4) for v in p), " ".join(used), "survey" if fixz else "solved"])
    with open(os.path.join(out, "link_offsets_geometry.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["position", "tag", "anchor", "mean_mapped_m", "consistent_m", "offset_removed_cm"])
        for (pos, tg), (p, used) in sorted(solved.items()):
            for an in used:
                k = (pos, tg, an)
                geo = float(np.linalg.norm(p - np.array(anc_xyz[an])))
                w.writerow([pos, tg, an, round(float(mapped[k].mean()), 4), round(geo, 4),
                            round((float(mapped[k].mean()) - geo) * 100, 2)])
    rows, summ = [], []
    for k in sorted(recs):
        pos, tg, an = k
        tr = truth[k]
        for i, (t, rssi, x) in enumerate(recs[k]):
            row = {"position": pos, "received_at": t, "tag": tg, "tag_hw": HW[tg], "anchor": an,
                   "anchor_hw": HW[an], "rssi_dbm": rssi, "range_raw_m": x,
                   "range_firstpath_m": round(float(fp[k][i]), 4),
                   "range_mapped_m": round(float(mapped[k][i]), 4),
                   "range_final_m": round(float(final[k][i]), 4),
                   "true_range_m": round(tr, 4) if tr is not None else ""}
            for name, v in (("raw", x), ("mapped", mapped[k][i]), ("final", final[k][i])):
                row[f"err_{name}_cm"] = round(float(v - tr) * 100, 2) if tr is not None else ""
            rows.append(row)
        if tr is not None:
            s = {"position": pos, "tag": tg, "tag_hw": HW[tg], "anchor": an, "n": len(recs[k]),
                 "true_range_m": round(tr, 4)}
            for name, v in (("raw", raw[k]), ("mapped", mapped[k]), ("final", final[k])):
                e = (v - tr) * 100
                s[f"{name}_bias_cm"] = round(float(e.mean()), 1)
                s[f"{name}_std_cm"] = round(float(e.std(ddof=1)) if len(e) > 1 else 0.0, 1)
                s[f"{name}_rmse_cm"] = round(float(math.sqrt((e ** 2).mean())), 1)
            summ.append(s)
    for name, data in (("ranges_fixed_all_tags.csv", rows), ("link_summary_fixed.csv", summ)):
        with open(os.path.join(out, name), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(data[0]))
            w.writeheader()
            w.writerows(data)
    print(f"wrote {len(rows)} readings -> {out}  (tag height: {'survey' if fixz else 'solved in 3D'})")

    # ---- results
    groups = [("T1", lambda k: k[1] == "T1"), ("T2", lambda k: k[1] == "T2"), ("T3", lambda k: k[1] == "T3"),
              ("T4", lambda k: k[1] == "T4"), ("ESP32", lambda k: HW[k[1]] == "ESP32"),
              ("T5", lambda k: k[1] == "T5"), ("T6", lambda k: k[1] == "T6"),
              ("DWM", lambda k: HW[k[1]] == "DWM"), ("ALL", lambda k: True)]
    for title, ancs in (("anchors A1-A5", FIT_ANCHORS), ("anchor A6", {"A6"}), ("anchor A7 (ESP32)", {"A7"})):
        print(f"\n{title}, per reading: bias / RMSE cm, within ±10 cm   raw | step 1-2 (mapped) | step 3 (final)")
        for g, sel in groups:
            s2 = lambda k, sel=sel: sel(k) and k[2] in ancs
            if not any(s2(k) and truth[k] is not None for k in raw):
                continue
            print(f"  {g:<6}" + " | ".join("%+6.1f / %5.1f  %3.0f%%" % score(a, truth, s2) for a in (raw, mapped, final)))

    held_m, held_f = {}, {}
    for P in sorted({k[0] for k in raw}):
        mp = build_maps(fp, truth, exclude_pos=P)
        fpP = {k: v for k, v in fp.items() if k[0] == P}
        m, f, _ = pipeline(fpP, truth, anc_xyz, tag_z, mp)
        held_m.update(m)
        held_f.update(f)
    print("\nleave-one-position-out (maps from 4 positions, applied to the 5th): RMSE cm, within ±10 cm")
    for title, ancs in (("A1-A5", FIT_ANCHORS), ("A6", {"A6"}), ("A7", {"A7"})):
        for g, sel in (("ESP32", lambda k: HW[k[1]] == "ESP32"), ("DWM", lambda k: HW[k[1]] == "DWM"),
                       ("ALL", lambda k: True)):
            s2 = lambda k, sel=sel: sel(k) and k[2] in ancs
            a, b = score(held_m, truth, s2), score(held_f, truth, s2)
            print(f"  {title:<6}{g:<6} step 1-2 {a[1]:5.1f} {a[2]:3.0f}%   ->   step 3 {b[1]:5.1f} {b[2]:3.0f}%")


if __name__ == "__main__":
    main()

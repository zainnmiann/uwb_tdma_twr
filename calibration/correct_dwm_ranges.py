#!/usr/bin/env python3
"""Offline range correction for the DWM1001 tags (T5, T6) in UWB-Results.

Model (fitted on the 24 Sep 2026 survey runs, 5 positions x 2 tags x A1-A5,
validated leave-one-position-out):

    r_corrected = (1 - K) * r_raw - OFFSET[tag]

    K = 0.0239                     (range-proportional bias, ~2.4 cm per metre)
    OFFSET = {T5: -0.1776 m, T6: -0.1460 m}

then a centred moving median over WINDOW consecutive readings of the same
tag/anchor link (removes the multipath jumps).

Usage:
    python3 correct_dwm_ranges.py <UWB-Results dir> <output dir> [survey.xlsx]

Writes <output dir>/ranges_corrected.csv (one row per DWM-tag reading).
If the survey workbook is given, also adds true range / error columns for the
surveyed anchors A1-A5 and writes link_summary.csv.
"""
import csv
import glob
import json
import math
import os
import sys
from collections import defaultdict

K = 0.0239
OFFSET = {"T5": -0.1776, "T6": -0.1460}
WINDOW = 9  # readings (~2 s at ~4.4 Hz per link)
# DWM1001 anchors only (A7 is an ESP32 and is excluded; A6 is DWM but unsurveyed).
DWM_ANCHORS = {"A1", "A2", "A3", "A4", "A5", "A6", "A8"}


def correct(tag, r):
    return (1.0 - K) * r - OFFSET[tag]


def moving_median(xs, w):
    h = w // 2
    out = []
    for i in range(len(xs)):
        win = sorted(xs[max(0, i - h):i + h + 1])
        n = len(win)
        out.append(win[n // 2] if n % 2 else 0.5 * (win[n // 2 - 1] + win[n // 2]))
    return out


def survey_truth(xlsx, meta_anchors):
    """True tag positions and A1-A5 positions in the metadata frame.

    The survey frame is mirrored in x and shifted relative to the metadata
    frame; the transform is fitted on the five surveyed anchors UW1..UW5 ->
    A01..A05, and the surveyed anchor coordinates are used for A1-A5.
    """
    import cmath
    import openpyxl
    ws = openpyxl.load_workbook(xlsx, data_only=True).active
    S = {r[0]: r[1:] for r in list(ws.iter_rows(values_only=True))[1:] if r[0]}
    src = [complex(-S[f"UW{k}"][0], S[f"UW{k}"][1]) for k in range(1, 6)]
    dst = [complex(*meta_anchors[f"A0{k}"]["xyz"][:2]) for k in range(1, 6)]
    cs, cd = sum(src) / 5, sum(dst) / 5
    num = sum((d - cd) * (s - cs).conjugate() for s, d in zip(src, dst))
    rot = num / abs(num)
    t = cd - rot * cs

    def T(p):
        c = rot * complex(-p[0], p[1]) + t
        return (c.real, c.imag, p[2])

    anchors = {f"A{k}": T(S[f"UW{k}"]) for k in range(1, 6)}
    tags = {k: T(v) for k, v in S.items() if not k.startswith("UW")}
    return anchors, tags


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    src_dir, out_dir = sys.argv[1], sys.argv[2]
    xlsx = sys.argv[3] if len(sys.argv) > 3 else None
    os.makedirs(out_dir, exist_ok=True)

    rows = []
    for d in sorted(glob.glob(os.path.join(src_dir, "position*"))):
        pos = os.path.basename(d).split("-")[0].replace("position", "")
        meta = json.load(open(os.path.join(d, "metadata.json")))
        anchors = tags = None
        if xlsx:
            anchors, tags = survey_truth(xlsx, meta["anchors"])
        links = defaultdict(list)
        for line in open(os.path.join(d, "raw.jsonl")):
            r = json.loads(line)
            p = json.loads(r["line"].split(" ", 1)[1])
            if p["tag_label"] in OFFSET and p["anchor_label"] in DWM_ANCHORS:
                links[(p["tag_label"], p["anchor_label"])].append((r, p))
        for (tag, anc), recs in sorted(links.items()):
            raw = [p["range_m"] for _, p in recs]
            cor = [correct(tag, x) for x in raw]
            filt = moving_median(cor, WINDOW)
            true = None
            if tags and anc in anchors:
                true = math.dist(tags[f"{pos}{tag}"], anchors[anc])
            for (r, p), x, c, f in zip(recs, raw, cor, filt):
                row = {
                    "position": pos, "received_at": r["received_at"],
                    "tag": tag, "anchor": anc, "rssi_dbm": p["rssi"],
                    "range_raw_m": x,
                    "range_corrected_m": round(c, 4),
                    "range_corrected_filtered_m": round(f, 4),
                }
                if xlsx:
                    row["true_range_m"] = round(true, 4) if true is not None else ""
                    for k, v in (("raw", x), ("corrected", c), ("filtered", f)):
                        row[f"err_{k}_cm"] = round((v - true) * 100, 2) if true is not None else ""
                rows.append(row)

    with open(os.path.join(out_dir, "ranges_corrected.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    if xlsx:
        g = defaultdict(list)
        for r in rows:
            if r["true_range_m"] != "":
                g[(r["position"], r["tag"], r["anchor"])].append(r)
        out = []
        for (pos, tag, anc), rs in sorted(g.items()):
            def stats(key):
                e = [x[key] for x in rs]
                m = sum(e) / len(e)
                rmse = math.sqrt(sum(v * v for v in e) / len(e))
                sd = math.sqrt(sum((v - m) ** 2 for v in e) / (len(e) - 1)) if len(e) > 1 else 0.0
                return round(m, 1), round(sd, 1), round(rmse, 1)
            row = {"position": pos, "tag": tag, "anchor": anc, "n": len(rs),
                   "true_range_m": rs[0]["true_range_m"]}
            for k in ("raw", "corrected", "filtered"):
                row[f"{k}_bias_cm"], row[f"{k}_std_cm"], row[f"{k}_rmse_cm"] = stats(f"err_{k}_cm")
            out.append(row)
        with open(os.path.join(out_dir, "link_summary.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(out[0]))
            w.writeheader()
            w.writerows(out)
    print(f"wrote {len(rows)} readings to {out_dir}")


if __name__ == "__main__":
    main()

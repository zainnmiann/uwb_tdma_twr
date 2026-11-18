#!/usr/bin/env python3
"""Final range-bias correction for all tags (T1-T6) of the 24 Sep 2026 survey runs.

Method (default, --method map): DW1000 APS011 calibration map, one per tag.
    Bias is ~0 around the antenna-delay calibration distance, negative closer
    in, positive further out, and not linear, so a constant (antenna-delay
    style) correction cannot remove it. For each tag, every reading on the
    surveyed anchors A1-A5 is binned by MEASURED distance (4 m bins from 4 m);
    each bin gives one table point:
        (mean measured distance, mean bias = mean(measured - true))
    Between table points the bias is interpolated with straight segments,
    beyond the ends it is held at the end value, and each reading is corrected
    with the bias looked up at its own measured range:
        r_corrected = r_measured - map_tag(r_measured)
    The tables are written to calibration_map.csv. The script also scores the
    maps leave-one-position-out (tables rebuilt from 4 positions, applied to
    the 5th).

Alternative (--method bench): DWM tags use the 18 Nov 2026 bench map
    (bench/, no survey input), ESP32 tags one constant
    offset each.

Every reading also gets a centred moving median (9 readings per link).

Truth: tag positions are the surveyed points (1T1..5T6) moved into the
metadata frame. Anchor positions are the METADATA coordinates (A1-A7, the
default since 18 Nov 2026): the survey and metadata disagree on anchor height
by up to 15 cm (A1 -9, A3 -15 cm in metadata), and the metadata set scored
better leave-one-position-out. --anchors survey uses the surveyed A1-A5
instead (A6/A7 then have no truth).

Usage:
    python3 correct_all_tags.py <UWB-Results dir> <survey.xlsx> <out dir>
                                [--anchors metadata|survey] [--method map|bench]
"""
import csv, glob, json, math, os, sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "bench"))
from correct_dwm_ranges import survey_truth, moving_median          # noqa: E402
from apply_bias_map import load_map, interp, CAL, HW               # noqa: E402

DWM_TAGS = {"T5", "T6"}
ESP_TAGS = {"T1", "T2", "T3", "T4"}
TAGS = sorted(DWM_TAGS | ESP_TAGS)
ANCHORS = {"A1", "A2", "A3", "A4", "A5", "A6", "A7"}
FIT_ANCHORS = {"A1", "A2", "A3", "A4", "A5"}   # calibration tables are built from these links only
BIN_EDGES = [4.0, 8.0, 12.0, 16.0, 20.0]       # measured-distance bins (m)
MIN_BIN = 50                                   # readings needed for a bin to become a table point
WINDOW = 9
BENCH = os.path.join(HERE, "bench", "summary.csv")


def build_maps(links, truth, exclude_pos=None):
    """tag -> [(mean measured m, mean bias m, n readings, bin lo, bin hi)], sorted."""
    acc = defaultdict(lambda: defaultdict(list))
    for (pos, tg, an), recs in links.items():
        tr = truth[(pos, tg, an)]
        if an not in FIT_ANCHORS or tr is None or pos == exclude_pos:
            continue
        for _, _, x in recs:
            for i in range(len(BIN_EDGES) - 1):
                if BIN_EDGES[i] <= x < BIN_EDGES[i + 1]:
                    acc[tg][i].append((x, x - tr))
                    break
    maps = {}
    for tg, bins in acc.items():
        maps[tg] = sorted((sum(m for m, _ in v) / len(v), sum(e for _, e in v) / len(v), len(v),
                           BIN_EDGES[i], BIN_EDGES[i + 1])
                          for i, v in bins.items() if len(v) >= MIN_BIN)
    return maps


def map_bias(tbl, r):
    return interp(r, [p[0] for p in tbl], [p[1] for p in tbl])


def main():
    args = sys.argv[1:]
    opt = {"--anchors": "metadata", "--method": "map"}
    for k in opt:
        if k in args:
            i = args.index(k)
            opt[k] = args[i + 1]
            del args[i:i + 2]
    mode, method = opt["--anchors"], opt["--method"]
    if len(args) != 3 or mode not in ("metadata", "survey") or method not in ("map", "bench"):
        sys.exit(__doc__)
    src, xlsx, out = args
    os.makedirs(out, exist_ok=True)

    # read everything once
    links = defaultdict(list)                    # (pos, tag, anchor) -> [(received_at, rssi, range)]
    truth = {}
    for d in sorted(glob.glob(os.path.join(src, "position*"))):
        pos = os.path.basename(d).split("-")[0].replace("position", "")
        meta_anc = json.load(open(os.path.join(d, "metadata.json")))["anchors"]
        anc_xyz, tag_xyz = survey_truth(xlsx, meta_anc)
        if mode == "metadata":
            anc_xyz = {f"A{k[1:].lstrip('0')}": tuple(v["xyz"]) for k, v in meta_anc.items()}
        for line in open(os.path.join(d, "raw.jsonl")):
            r = json.loads(line)
            p = json.loads(r["line"].split(" ", 1)[1])
            tg, an = p["tag_label"], p["anchor_label"]
            if tg not in TAGS or an not in ANCHORS:
                continue
            links[(pos, tg, an)].append((r["received_at"], p["rssi"], p["range_m"]))
            if (pos, tg, an) not in truth:
                truth[(pos, tg, an)] = math.dist(tag_xyz[f"{pos}{tg}"], anc_xyz[an]) if an in anc_xyz else None

    if method == "map":
        maps = build_maps(links, truth)
        with open(os.path.join(out, "calibration_map.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["tag", "tag_hw", "bin_lo_m", "bin_hi_m", "n_readings", "mean_measured_m", "bias_cm"])
            for tg in TAGS:
                for m, b, n, lo, hi in maps[tg]:
                    w.writerow([tg, HW[tg], lo, hi, n, round(m, 4), round(b * 100, 2)])
        print("calibration maps (mean measured m -> bias cm):")
        for tg in TAGS:
            print(f"  {tg} ({HW[tg]:>5}): " + "   ".join(f"{m:6.2f} {b * 100:+6.1f}" for m, b, *_ in maps[tg]))

        def correction(tg, an, r):
            return map_bias(maps[tg], r), "calibration map (APS011)"
    else:
        xs, ys, _ = load_map(BENCH)
        bmap = lambda d: interp(d, xs, ys)
        acc = defaultdict(list)
        for (pos, tg, an), recs in links.items():
            if tg in ESP_TAGS and an in FIT_ANCHORS and truth[(pos, tg, an)] is not None:
                acc[tg] += [x - truth[(pos, tg, an)] for _, _, x in recs]
        esp_off = {tg: sum(v) / len(v) for tg, v in sorted(acc.items())}
        json.dump({tg: round(v, 4) for tg, v in esp_off.items()},
                  open(os.path.join(out, "esp32_tag_offsets_m.json"), "w"), indent=2)

        def correction(tg, an, r):
            if tg in DWM_TAGS:
                zero = bmap(CAL[an]) + (bmap(CAL[tg]) if tg != "T1" else 0.0)
                return bmap(r) - zero, "bench map (APS011)"
            return esp_off[tg], "ESP32 tag offset (survey)"

    rows, summ = [], []
    for (pos, tg, an), recs in sorted(links.items()):
        tr = truth[(pos, tg, an)]
        cor, meth = [], None
        for _, _, x in recs:
            c, meth = correction(tg, an, x)
            cor.append(x - c)
        filt = moving_median(cor, WINDOW)
        e = {"raw": [], "corrected": [], "filtered": []}
        for (t, rssi, x), c, f in zip(recs, cor, filt):
            row = {"position": pos, "received_at": t, "tag": tg, "tag_hw": HW[tg], "anchor": an,
                   "anchor_hw": HW[an], "rssi_dbm": rssi, "range_raw_m": x, "method": meth,
                   "correction_m": round(x - c, 4), "range_corrected_m": round(c, 4),
                   "range_corrected_filtered_m": round(f, 4),
                   "true_range_m": round(tr, 4) if tr is not None else ""}
            for k, v in (("raw", x), ("corrected", c), ("filtered", f)):
                row[f"err_{k}_cm"] = round((v - tr) * 100, 2) if tr is not None else ""
                if tr is not None:
                    e[k].append((v - tr) * 100)
            rows.append(row)
        if tr is not None:
            s = {"position": pos, "tag": tg, "tag_hw": HW[tg], "anchor": an, "n": len(recs),
                 "true_range_m": round(tr, 4), "method": meth}
            for k, v in e.items():
                m = sum(v) / len(v)
                s[f"{k}_bias_cm"] = round(m, 1)
                s[f"{k}_std_cm"] = round(math.sqrt(sum((x - m) ** 2 for x in v) / max(1, len(v) - 1)), 1)
                s[f"{k}_rmse_cm"] = round(math.sqrt(sum(x * x for x in v) / len(v)), 1)
            summ.append(s)

    for name, data in (("ranges_corrected_all_tags.csv", rows), ("link_summary_all_tags.csv", summ)):
        with open(os.path.join(out, name), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(data[0]))
            w.writeheader()
            w.writerows(data)
    print(f"wrote {len(rows)} readings, {len(summ)} links with truth -> {out}")

    def line(label, errs_by_kind):
        cells = []
        for v in errs_by_kind:
            cells.append((sum(v) / len(v), math.sqrt(sum(x * x for x in v) / len(v)),
                          100 * sum(abs(x) <= 10 for x in v) / len(v)))
        return (f"{label:<6}{len(errs_by_kind[0]):>6}   " + "   ".join(f"{b:+6.1f} / {r:5.1f}" for b, r, _ in cells)
                + "   " + " -> ".join(f"{p:.0f}%" for _, _, p in cells))

    groups = ["T1", "T2", "T3", "T4", "ESP32", "T5", "T6", "DWM", "ALL"]
    member = lambda r, g: g == "ALL" or r["tag"] == g or r["tag_hw"] == g
    for title, ancs in ((f"anchors A1-A5 ({mode} coordinates)", FIT_ANCHORS), ("anchor A6", {"A6"}),
                        ("anchor A7 (ESP32)", {"A7"})):
        sel0 = [r for r in rows if r["true_range_m"] != "" and r["anchor"] in ancs]
        if not sel0:
            continue
        print(f"\n{title}, per reading (cm):")
        print(f"{'tag':<6}{'n':>6}   {'raw bias/RMSE':>15}   {'corrected':>15}   {'+ median':>15}   ±10cm raw->cor->med")
        for g in groups:
            sel = [r for r in sel0 if member(r, g)]
            if sel:
                print(line(g, [[r[f"err_{k}_cm"] for r in sel] for k in ("raw", "corrected", "filtered")]))

    if method == "map":
        # leave-one-position-out: tables from 4 positions, applied to the 5th (A1-A5)
        held = defaultdict(list)
        for P in sorted({k[0] for k in links}):
            mp = build_maps(links, truth, exclude_pos=P)
            for (pos, tg, an), recs in links.items():
                tr = truth[(pos, tg, an)]
                if pos != P or an not in FIT_ANCHORS or tr is None:
                    continue
                held[tg] += [(x - map_bias(mp[tg], x) - tr) * 100 for _, _, x in recs]
        print("\nleave-one-position-out check (A1-A5, per reading, cm):")
        for g in groups:
            v = [x for tg in TAGS if g == "ALL" or tg == g or HW[tg] == g for x in held[tg]]
            print(f"  {g:<6} bias {sum(v) / len(v):+5.1f}  RMSE {math.sqrt(sum(x * x for x in v) / len(v)):5.1f}  "
                  f"within ±10 cm {100 * sum(abs(x) <= 10 for x in v) / len(v):3.0f}%")


if __name__ == "__main__":
    main()

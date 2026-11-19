# Range-bias calibration map (DW1000 APS011)

Antenna-delay calibration zeroes the range at ONE distance. DW1000 range bias
is ~0 around that calibration distance, negative closer in, positive further
out, and not linear, so a constant (antenna-delay) correction cannot remove
it. This folder removes it with a calibration map.

## Method

For each tag, readings with a known true range are grouped into bins by
**measured** distance. Each bin gives one table point:

    (mean measured distance, mean bias = mean(measured - true))

Each reading is corrected by subtracting the bias looked up at its own
measured distance, with straight-line interpolation between table points and
the end value held beyond the ends:

    r_corrected = r_measured - map_tag(r_measured)

## Files

| path | what |
|---|---|
| `fix_recorded_ranges.py` | **Full fix of the recorded ranges**: (1) first-path filter (20th percentile of 9 readings), (2) per-tag calibration map, (3) geometric consistency: each static tag's ranges to all 7 anchors are made to meet at one robustly solved 3D point, using anchor coordinates only. A1–A5 RMSE 26.9 → 7.3 cm, 87% within ±10 cm (leave-one-position-out 7.9 cm / 84%). Step 3 makes a tag's ranges consistent with that point, so don't use them to judge positioning accuracy. |
| `correct_all_tags.py` | Final correction of the 24 Sep 2026 survey runs, all tags T1–T6. Builds one map per tag from the surveyed links (4 m bins), applies it, adds a 9-reading moving median, and checks the maps leave-one-position-out. `--method bench` uses the bench map instead. |
| `correct_dwm_ranges.py` | Survey helpers (`survey_truth`, `moving_median`) plus the earlier DWM-only correction. |
| `results/sep24_calibration_map.csv` | The per-tag tables produced for the 24 Sep 2026 data. |
| `results/sep24_calibration_map_firstpath.csv` | The bias map applied by `fix_recorded_ranges.py` (step 2): per tag, mean measured distance → bias in 4 m bins, built on the first-path-filtered ranges. |
| `results/sep24_link_offsets_geometry.csv` | The per-link bias removed by `fix_recorded_ranges.py` step 3: each link's mean after step 2, the distance consistent with the solved tag point, and the difference. |
| `bench/calib_log.py` | Logs one tag→anchor link at a measured distance (tag on USB, anchor on a power bank) and appends the mean measured range and bias to `bench/summary.csv`. |
| `bench/apply_bias_map.py` | Builds the map from the bench runs and applies it, re-zeroed per link at the distances where that tag and anchor had their antenna delays calibrated. Also writes the arrays it applied: `bench_calibration_map.csv` and `bench_map_applied_per_link.csv` (each link's zero offset and the map values after subtracting it). |
| `bench/bench_calibration_map.csv` | The bench calibration array: mean measured distance → bias at 1.5 / 5.5 / 9 / 12.5 / 16 m (runs at the same distance pooled). |
| `bench/summary.csv`, `bench/data/` | Bench runs from 18 Nov 2026: two DWM1001s on this firmware (Sep 24 config, hash 0x02F3, preamble 256), line of sight, at 1.5 / 5.5 / 9 (×3) / 12.5 / 16 m, ~520 readings each. |

## Running

    python3 correct_all_tags.py <UWB-Results dir> <survey.xlsx> <out dir>
    python3 bench/calib_log.py 5.500 -n "note"          # one bench point

`correct_all_tags.py` and `fix_recorded_ranges.py` need `openpyxl` (survey workbook); `fix_recorded_ranges.py` also needs `numpy`.

    python3 fix_recorded_ranges.py <UWB-Results dir> <survey.xlsx> <out dir>

## Result on the 24 Sep 2026 runs

Truth: surveyed tag points, metadata anchor coordinates. Per reading on A1–A5:

| | raw bias / RMSE | corrected bias / RMSE | within ±10 cm |
|---|---|---|---|
| ESP32 tags (T1–T4) | +27.3 / 31.5 cm | 0.0 / 13.1 cm | 6% → 65% |
| DWM1001 tags (T5–T6) | +5.8 / 13.8 cm | +0.1 / 10.7 cm | 57% → 77% |
| all tags | +20.1 / 26.9 cm | +0.1 / 12.4 cm | 23% → 69% |

The maps are built from the survey, so those numbers are in-sample. Built
from 4 positions and applied to the 5th: RMSE 13.2 cm, 68% within ±10 cm
(DWM 11.5 cm / 75%, ESP32 14.0 cm / 64%).

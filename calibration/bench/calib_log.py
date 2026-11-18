#!/usr/bin/env python3
"""Log tag->anchor ranges at one measured distance (range-bias calibration).

Only the TAG is on USB. The anchor just needs power; it starts ranging by
itself from its saved config.

    /usr/bin/python3 calib_log.py 5.000                 # 120 s at 5.000 m
    /usr/bin/python3 calib_log.py 5.000 -s 180 -n "LOS, anchor 3.1 m high"

Writes, next to this script:
    data/d<dist>m_<time>.csv   one row per range (raw fields from the tag)
    summary.csv                one row per run (appended)

Ctrl-C stops early and still writes the summary.
"""
import argparse, csv, glob, math, os, statistics as st, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from uwb import check_line
import serial

TAG_SN = "760131635"                 # T1 board, calibration tag
F_SS, F_NLOS, F_RESP_LOST, F_REP_LOST = 0x01, 0x02, 0x04, 0x08
HERE = os.path.dirname(os.path.abspath(__file__))


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem000*"))
    for p in ports:
        if TAG_SN in p:
            return p
    if len(ports) == 1:
        return ports[0]
    sys.exit(f"tag {TAG_SN} not found; connected: {ports or 'none'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("distance_m", type=float, help="measured antenna-to-antenna distance")
    ap.add_argument("-s", "--seconds", type=float, default=120)
    ap.add_argument("-n", "--note", default="")
    ap.add_argument("-p", "--port")
    a = ap.parse_args()

    port = a.port or find_port()
    os.makedirs(os.path.join(HERE, "data"), exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = os.path.join(HERE, "data", f"d{a.distance_m:.3f}m_{stamp}.csv")
    cols = ["host_time", "sfn", "frame", "tag", "anchor", "slot", "d_mm",
            "tof_ps", "rxp_dbm", "fpp_dbm", "flags", "t_local_us"]

    s = serial.Serial(port, 115200, timeout=0.2)
    s.reset_input_buffer()
    good, flags_seen, buf = [], {"ss": 0, "nlos": 0, "lost": 0}, b""
    t0 = last = time.time()
    print(f"logging {port} at {a.distance_m:.3f} m for {a.seconds:.0f} s -> {path}")
    try:
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(cols)
            while time.time() - t0 < a.seconds:
                buf += s.read(4096)
                *lines, buf = buf.split(b"\n")
                for raw in lines:
                    body = check_line(raw.decode("ascii", "replace").strip())
                    if not body or not body.startswith("RNG,"):
                        continue
                    f = body.split(",")[1:]
                    if len(f) != 11 or f[3] == "0x0000":
                        continue                      # empty slot (no anchor there)
                    fl = int(f[9], 16)
                    w.writerow([f"{time.time():.3f}"] + f)
                    if fl & (F_RESP_LOST | F_REP_LOST):
                        flags_seen["lost"] += 1
                        continue
                    flags_seen["ss"] += bool(fl & F_SS)
                    flags_seen["nlos"] += bool(fl & F_NLOS)
                    good.append((int(f[5]) / 1000.0, float(f[7]), float(f[8]), fl))
                if time.time() - last >= 10:
                    last = time.time()
                    r = [g[0] for g in good[-40:]]
                    print(f"  {last - t0:5.0f} s  n={len(good):4d}  last {r[-1] if r else float('nan'):.3f} m  "
                          f"recent mean {st.mean(r) if r else float('nan'):.3f} m")
                    fh.flush()
    except KeyboardInterrupt:
        print("stopped early")
    s.close()

    if len(good) < 2:
        sys.exit("fewer than 2 good ranges -- is the anchor powered and in range?")
    r = [g[0] for g in good]
    rx = [g[1] for g in good]
    fp = [g[2] for g in good]
    mean, med, sd = st.mean(r), st.median(r), st.stdev(r)
    row = {
        "time": stamp, "true_m": f"{a.distance_m:.3f}", "n": len(r),
        "mean_m": f"{mean:.4f}", "median_m": f"{med:.4f}", "sd_cm": f"{sd * 100:.2f}",
        "bias_cm": f"{(mean - a.distance_m) * 100:.2f}",
        "bias_median_cm": f"{(med - a.distance_m) * 100:.2f}",
        "rxp_dbm": f"{st.mean(rx):.1f}", "fpp_dbm": f"{st.mean(fp):.1f}",
        "nlos_pct": f"{100 * flags_seen['nlos'] / len(r):.1f}",
        "ss_fallback_pct": f"{100 * flags_seen['ss'] / len(r):.1f}",
        "lost": flags_seen["lost"], "note": a.note, "file": os.path.basename(path),
    }
    summ = os.path.join(HERE, "summary.csv")
    new = not os.path.exists(summ)
    with open(summ, "a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)
    print(f"\n{len(r)} ranges at {a.distance_m:.3f} m: mean {mean:.4f}  median {med:.4f}  "
          f"SD {sd * 100:.1f} cm  ->  bias {(mean - a.distance_m) * 100:+.1f} cm")
    print(f"rxp {st.mean(rx):.1f} dBm  fpp {st.mean(fp):.1f} dBm  "
          f"NLOS {row['nlos_pct']}%  SS-fallback {row['ss_fallback_pct']}%  lost {flags_seen['lost']}")


if __name__ == "__main__":
    main()

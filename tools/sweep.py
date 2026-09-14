#!/usr/bin/env python3
"""Walk resp_pitch_us down across all four boards, measuring where it breaks.

For each pitch: reconfigure all four, reboot, clear counters, capture, record.
Writes results to measurements/pitch_sweep_<date>.json
"""
import sys, os, time, json, subprocess, datetime
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from uwb import BOARDS, popen, cmd, get_stat, get_measure, get_cfg
import set_common, collect

PITCHES = [int(x) for x in sys.argv[1:]] or [1895, 1400, 1100, 900, 700, 615, 500]
CAPTURE_S = 10.0
OUT = os.path.join(HERE, "..", "measurements",
                   f"pitch_sweep_{datetime.date.today()}.json")


def clear_counters():
    for name in BOARDS:
        port, baud = BOARDS[name]
        try:
            s = popen(port, baud, settle=0.2)
            cmd(s, "STAT,CLEAR")
            s.close()
        except Exception as e:
            print(f"  {name}: clear failed {e}")


def main():
    results = {}
    for pitch in PITCHES:
        print(f"\n{'=' * 72}\n=== resp_pitch_us = {pitch} ===\n{'=' * 72}")
        set_common.RESP_PITCH = pitch
        set_common.REP_PITCH = pitch
        set_common.COMMON["resp_pitch_us"] = pitch
        set_common.COMMON["report_pitch_us"] = pitch

        hashes, ok = {}, True
        for name in BOARDS:
            good, log, cfg = set_common.configure(name)
            if not good or not cfg:
                print(f"  {name}: CONFIG FAILED")
                for l in log[-4:]:
                    print("    " + l)
                ok = False
                continue
            hashes[name] = cfg.get("_hash")
            print(f"  {name:9} hash={cfg.get('_hash')} "
                  f"T_f={cfg.get('frame_dur_us')} dr={cfg.get('resp_pitch_us')}")
        uniq = set(h for h in hashes.values() if h)
        if len(uniq) != 1:
            print(f"  !! hash mismatch {uniq} -- skipping")
            results[pitch] = {"error": f"hash mismatch {uniq}"}
            continue

        time.sleep(5)            # let all four reboot and re-sync
        clear_counters()
        time.sleep(3)            # accumulate fresh counters
        try:
            res = collect.main.__wrapped__() if hasattr(collect.main, "__wrapped__") \
                  else run_capture()
        except Exception as e:
            print(f"  capture error: {e}")
            res = {}
        results[pitch] = {"hash": uniq.pop(), "capture": res}
        with open(OUT, "w") as f:
            json.dump(results, f, indent=1, default=str)
    print(f"\nwrote {OUT}")
    return results


def run_capture():
    collect.SECS = CAPTURE_S
    return collect.main()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Put all four boards on one common config so a MIXED network can form.

Usage:  set_common.py [resp_pitch] [report_pitch]
Defaults to the ESP32's known-safe pitches (1895/1891).

Per board: $RANGE,STOP -> widen frame_dur -> set keys -> read the firmware's
own required frame_dur from $TABLEB -> set it -> $SAVE -> $RESET.
"""
import sys, time, json, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb import BOARDS, popen, cmd, query, get_cfg

RESP_PITCH = int(sys.argv[1]) if len(sys.argv) > 1 else 1895
REP_PITCH = int(sys.argv[2]) if len(sys.argv) > 2 else 1891

# frame_index / slot_index are already unique across the four boards.
ROLE_KEYS = {
    "DWM_ANC": {"role": "RESP", "slot_index": 0},
    "DWM_TAG": {"role": "INIT", "frame_index": 0},
    "ESP_ANC": {"role": "RESP", "slot_index": 1},
    "ESP_TAG": {"role": "INIT", "frame_index": 1},
}

COMMON = {
    "init_count": 2,
    "resp_count": 2,
    "rate_div": 1,
    "report_enable": 1,
    "guard_us": 400,
    "rx_timeout_us": 1200,
    "resp_base_us": 1683,
    "resp_pitch_us": RESP_PITCH,
    "report_base_us": 1749,
    "report_pitch_us": REP_PITCH,
    "out_format": "CSV",
    "out_mask": 3,
}

WIDE = 200000          # max frame_dur_us; always satisfies eq (14) mid-edit


def required_frame_dur(s):
    for l in query(s, "TABLEB", ("frame_dur_us_required",)):
        p = l.split("*")[0].split(",")
        if "frame_dur_us_required" in p:
            return int(p[p.index("frame_dur_us_required") + 1])
    return None


def configure(name):
    port, baud = BOARDS[name]
    s = popen(port, baud)
    log = []

    def do(c):
        r = cmd(s, c)
        ok = bool(r) and r[0].startswith("$OK")
        log.append(f"{'ok ' if ok else 'ERR'} {c}  -> {r[0] if r else '(no reply)'}")
        return ok

    cmd(s, "RANGE,STOP")
    do(f"SET,frame_dur_us,{WIDE}")
    for k, v in {**COMMON, **ROLE_KEYS[name]}.items():
        do(f"SET,{k},{v}")

    req = required_frame_dur(s)
    if req is None:
        log.append("ERR could not read frame_dur_us_required")
        s.close()
        return False, log, None
    do(f"SET,frame_dur_us,{req}")

    saved = cmd(s, "SAVE")
    log.append(f"SAVE -> {saved[0] if saved else '(no reply)'}")
    cfg = get_cfg(s)
    log.append(f"hash={cfg.get('_hash')} frame_dur={cfg.get('frame_dur_us')} "
               f"dr={cfg.get('resp_pitch_us')} N_R={cfg.get('resp_count')} "
               f"D={cfg.get('rate_div')}")
    cmd(s, "RESET", want=("$OK", "$I", "$ERR"), attempts=1, window=0.6)
    s.close()
    return True, log, cfg


def main():
    print(f"=== common config: resp_pitch={RESP_PITCH} report_pitch={REP_PITCH} ===")
    cfgs = {}
    for name in BOARDS:
        print(f"\n--- {name} ---")
        ok, log, cfg = configure(name)
        for l in log:
            print("  " + l)
        if cfg:
            cfgs[name] = cfg
    print("\n=== hashes ===")
    hs = {n: c.get("_hash") for n, c in cfgs.items()}
    for n, h in hs.items():
        print(f"  {n:9} {h}")
    uniq = set(h for h in hs.values() if h)
    print(f"  -> {'ALL MATCH' if len(uniq) == 1 else 'MISMATCH: ' + str(uniq)}")
    return cfgs


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Apply the settled mixed-network config to all four boards:
  - resp_pitch_us/report_pitch_us = 1100 (measured safe floor, this session)
  - clear each board's $ANCHOR list (all en=0) so anchor_id falls through to
    the observed MHR source address instead of a stale provisioned entry
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb import BOARDS, popen, cmd, get_cfg
import set_common

PITCH = 1100

ROLE_KEYS = {
    "DWM_ANC": {"role": "RESP", "slot_index": 0},
    "DWM_TAG": {"role": "INIT", "frame_index": 0},
    "ESP_ANC": {"role": "RESP", "slot_index": 1},
    "ESP_TAG": {"role": "INIT", "frame_index": 1},
}
set_common.ROLE_KEYS = ROLE_KEYS
set_common.RESP_PITCH = PITCH
set_common.REP_PITCH = PITCH
set_common.COMMON.update({"resp_pitch_us": PITCH, "report_pitch_us": PITCH})


def clear_anchor_list(s, log):
    # 12 slots (CFG_MAX_RESP), all disabled -- id 0 keeps them inert.
    for i in range(12):
        r = cmd(s, f"ANCHOR,{i},0,{i},0,0,0,0,0")
        if not r or not r[0].startswith("$OK"):
            log.append(f"  ANCHOR,{i} -> {r[0] if r else '(no reply)'}")


def main():
    print(f"=== settled config: resp_pitch=report_pitch={PITCH}, anchor lists cleared ===")
    for name in BOARDS:
        print(f"\n--- {name} ---")
        port, baud = BOARDS[name]
        s = popen(port, baud)
        cmd(s, "RANGE,STOP")
        clear_anchor_list(s, [])
        s.close()

        ok, log, cfg = set_common.configure(name)
        for l in log[-3:]:
            print("  " + l)
        if cfg:
            print(f"  hash={cfg.get('_hash')} dr={cfg.get('resp_pitch_us')} "
                  f"N_R={cfg.get('resp_count')}")

    print("\n=== verifying ===")
    time.sleep(4)
    hashes = {}
    for name in BOARDS:
        port, baud = BOARDS[name]
        s = popen(port, baud, settle=0.3)
        cfg = get_cfg(s)
        hashes[name] = cfg.get("_hash")
        print(f"  {name:9} hash={cfg.get('_hash')} pitch={cfg.get('resp_pitch_us')}")
        s.close()
    uniq = set(hashes.values())
    print(f"  -> {'ALL MATCH' if len(uniq) == 1 else 'MISMATCH ' + str(uniq)}")


if __name__ == "__main__":
    main()

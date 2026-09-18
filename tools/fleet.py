#!/usr/bin/env python3
"""Provision one board into the 6-tag / 8-anchor fleet, or verify the fleet.

    python3 tools/fleet.py <port> tag <0-5>        # role INIT, frame_index N
    python3 tools/fleet.py <port> anchor <0-7>     # role RESP, slot_index N
    python3 tools/fleet.py --verify                # check every board in BOARDS
    python3 tools/fleet.py --plan                  # print the target config

Every node in a network must agree on the hashed parameters or its frames are
dropped on a cfg_hash mismatch -- so provisioning is all-or-nothing, and this
script exists to make it mechanical rather than hand-typed 14 times.

A tag's TDMA position is frame_index (validated < init_count); an anchor's is
slot_index (validated < resp_count). Every board reports BOTH fields, so the
wrong one looks like it was accepted and silently does nothing. Order matters
for the same reason: init_count/resp_count are raised before the index is set,
or the index fails validation against the old, smaller count.

frame_dur_us is never hard-coded. It is widened to a large value first (so the
per-slot keys can be loosened without tripping eq(14)), then read back from the
firmware's own $TABLEB as frame_dur_us_required. The expected value is asserted
afterwards, so a firmware change that alters the air model is caught here
instead of silently reshaping the superframe.
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb import BOARDS, popen, cmd, get_cfg

N_TAGS = 6
N_ANCHORS = 8

# Hashed parameters. Derived on hardware 2026-09-18 from $TABLEB at N_I=6,
# N_R=8, preamble_len=256: eq(12) needs delta_r>=1276 / delta_p>=1272 and
# eq(13) needs T_r0>=1814 / T_p0>=1880. The values below clear those with a
# small margin. The 2-anchor set (resp_base 700, pitches 1100) does NOT hold
# at 8 anchors -- $TABLEB reports eq12/eq13 TIGHT -- because the Report frame
# now carries 8 timestamp sets.
#
# preamble_len stays 256 deliberately. Dropping to 128 relaxes the minima to
# 1683/1145/1749/1141 and shortens the frame to T_f=22141 (7.5 Hz vs 6.7 Hz),
# only a 12% gain: at N_R=8 the frame is dominated by its 16 slots, and each
# slot's pitch is set by processing time, not preamble air time. 256 is what
# raised anchor Poll reception to 98-99% in the mixed fleet, so it is not
# worth trading for 0.8 Hz.
COMMON = {
    "init_count":      N_TAGS,
    "resp_count":      N_ANCHORS,
    "preamble_len":    256,
    "guard_us":        300,
    "resp_base_us":    1850,
    "resp_pitch_us":   1300,
    "report_base_us":  1900,
    "report_pitch_us": 1300,
    "rx_timeout_us":   900,
    "rate_div":        1,
    "report_enable":   1,
    "sync_mmax":       10,
}

# rx_timeout_us MUST stay well below resp_pitch_us. Measured 2026-09-17: at
# rx_timeout 1200 against resp_pitch 1244, slot 0's RX window stayed open
# until 2150us while Response[1] arrived at 2194us, so slot 0 swallowed slot
# 1's frame and resp_lost doubled to 8.7%.
assert COMMON["rx_timeout_us"] < COMMON["resp_pitch_us"] - 200, "rx_timeout too close to resp_pitch"

EXPECT_FRAME_DUR = 24892
EXPECT_HASH = "0x650D"
SUPERFRAME_US = EXPECT_FRAME_DUR * N_TAGS


def _set(s, key, val):
    out = cmd(s, f"SET,{key},{val}", attempts=8, window=1.5)
    time.sleep(0.25)
    bad = [l for l in out if "ERR" in l]
    if bad:
        raise RuntimeError(f"SET {key}={val} rejected: {bad[0]}")


def _tableb(s):
    d, req, tight = {}, None, []
    for l in cmd(s, "TABLEB", want=("$TB,",), attempts=8, window=2.5):
        f = l.split("*")[0].split(",")[1:]
        d.update(dict(zip(f[0::2], f[1::2])))
        if "TIGHT" in l:
            tight.append(l.split("*")[0])
    if "frame_dur_us_required" in d:
        req = int(d["frame_dur_us_required"])
    return req, tight


def provision(port, role, index, baud=None):
    if role == "tag" and not 0 <= index < N_TAGS:
        raise SystemExit(f"tag index must be 0..{N_TAGS - 1}")
    if role == "anchor" and not 0 <= index < N_ANCHORS:
        raise SystemExit(f"anchor index must be 0..{N_ANCHORS - 1}")

    for b in ([baud] if baud else [921600, 115200]):
        s = popen(port, b, settle=1.2)
        if get_cfg(s):
            break
        s.close()
    else:
        raise SystemExit(f"{port}: no response at 921600 or 115200")

    cmd(s, "RANGE,STOP", attempts=10, window=2.0)
    time.sleep(0.4)
    # Widen first so the per-slot keys below can grow without tripping eq(14).
    _set(s, "frame_dur_us", 200000)

    _set(s, "role", "INIT" if role == "tag" else "RESP")
    # Counts before the index: the index is validated against them.
    _set(s, "init_count", N_TAGS)
    _set(s, "resp_count", N_ANCHORS)
    _set(s, "frame_index" if role == "tag" else "slot_index", index)

    for k, v in COMMON.items():
        if k not in ("init_count", "resp_count"):
            _set(s, k, v)

    req, tight = _tableb(s)
    if tight:
        raise RuntimeError("spec constraints not satisfied:\n  " + "\n  ".join(tight))
    if req != EXPECT_FRAME_DUR:
        raise RuntimeError(
            f"$TABLEB requires frame_dur_us={req}, expected {EXPECT_FRAME_DUR}. "
            "The air model changed -- update EXPECT_FRAME_DUR/EXPECT_HASH and "
            "reprovision EVERY node, not just this one.")
    _set(s, "frame_dur_us", req)

    cmd(s, "SAVE", attempts=10, window=2.5)
    time.sleep(0.6)
    c = get_cfg(s)
    s.close()

    label = f"T{index + 1}" if role == "tag" else f"A{index + 1}"
    ok = c.get("_hash") == EXPECT_HASH
    print(f"{label:>3}  {c.get('node_id')}  {port}")
    print(f"     role={c.get('role')} "
          f"{'frame_index' if role == 'tag' else 'slot_index'}={index} "
          f"T_f={c.get('frame_dur_us')} hash={c.get('_hash')} "
          f"{'OK' if ok else 'MISMATCH -- expected ' + EXPECT_HASH}")
    return ok


def verify():
    print(f"target hash {EXPECT_HASH}, T_f {EXPECT_FRAME_DUR} us, "
          f"superframe {SUPERFRAME_US / 1000:.1f} ms "
          f"-> {1e6 / SUPERFRAME_US:.1f} Hz per tag per anchor\n")
    bad = 0
    for name, (port, baud) in BOARDS.items():
        try:
            s = popen(port, baud, settle=0.8)
            c = get_cfg(s)
            s.close()
        except Exception as e:
            print(f"{name:>7}: UNREACHABLE ({type(e).__name__})")
            bad += 1
            continue
        if not c:
            print(f"{name:>7}: no $CFG response")
            bad += 1
            continue
        h = c.get("_hash")
        idx = c.get("frame_index") if c.get("role") == "INIT" else c.get("slot_index")
        mark = "ok" if h == EXPECT_HASH else f"MISMATCH (want {EXPECT_HASH})"
        if h != EXPECT_HASH:
            bad += 1
        print(f"{name:>7}: {c.get('node_id')} role={c.get('role'):>4} idx={idx} "
              f"N_I={c.get('init_count')} N_R={c.get('resp_count')} "
              f"T_f={c.get('frame_dur_us')} hash={h} {mark}")
    print(f"\n{'all nodes agree' if not bad else str(bad) + ' node(s) need attention'}")
    return bad == 0


def plan():
    print(f"fleet: {N_TAGS} tags x {N_ANCHORS} anchors")
    for k, v in COMMON.items():
        print(f"  {k:<16} = {v}")
    print(f"  {'frame_dur_us':<16} = {EXPECT_FRAME_DUR}  (from $TABLEB)")
    print(f"  {'cfg_hash':<16} = {EXPECT_HASH}")
    print(f"  superframe = {SUPERFRAME_US / 1000:.1f} ms "
          f"-> {1e6 / SUPERFRAME_US:.1f} Hz per tag per anchor")


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a or a[0] in ("-h", "--help"):
        print(__doc__)
    elif a[0] == "--verify":
        sys.exit(0 if verify() else 1)
    elif a[0] == "--plan":
        plan()
    elif len(a) >= 3 and a[1] in ("tag", "anchor"):
        sys.exit(0 if provision(a[0], a[1], int(a[2])) else 1)
    else:
        print(__doc__)
        sys.exit(2)

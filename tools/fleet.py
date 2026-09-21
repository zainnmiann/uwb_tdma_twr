#!/usr/bin/env python3
"""Provision one board into the 6-tag / 7-anchor fleet, or verify the fleet.

    python3 tools/fleet.py <port> tag <0-5>        # role INIT, frame_index N
    python3 tools/fleet.py <port> anchor <0-6>     # role RESP, slot_index N
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
N_ANCHORS = 7

# Hashed parameters. eq(12)/eq(13) minima (t_r0_us, delta_r_us, t_p0_us,
# delta_p_us in $TABLEB) are pure functions of cfg.proc_us/isr_us/margin_us/
# proc_poll_us/proc_final_us -- and those five fields are NOT hashed, so each
# board can carry its own. But the geometry they size (resp_base_us,
# resp_pitch_us, report_base_us, report_pitch_us) IS hashed and shared, and
# it is the ANCHOR that actually spends this budget arming its Response and
# Report. This fleet's anchors are a MIX of DWM1001 and ESP32 (A7), so the
# minima were computed 2026-09-18 by setting a board's proc_* fields to the
# nRF52 port's own defaults (proc_us=1000, isr_us=15, margin_us=100,
# proc_poll_us=1000, proc_final_us=1000 -- see port/nrf52/
# board_defaults.c) before reading $TABLEB, NOT from a connected tag's own
# (irrelevant) proc_* values -- a tag never responds or reports, so its
# $MEASURE proc_resp_us/proc_rep_us read 0 and its proc_us has no bearing
# on what an anchor needs.
#
# eq(12)/eq(13) minima do NOT depend on N_R (only eq(14)'s frame_dur_us
# does, via the (N_R-1) terms), so the same resp_base/pitch/report_base/
# pitch from the N_R=12 point carry over unchanged to N_R=7; only
# frame_dur_us was recomputed. delta_r>=1441, delta_p>=1437, t_r0>=1314,
# t_p0>=1373 -- all comfortably under the chosen 1471/1467/1364/1448.
#
# A7's own $MEASURE (2026-09-21, real live traffic, N_R=12 point) showed
# proc_resp_us max 595 / budget 1364 and proc_rep_us max 917 / budget 1448
# -- both fit with ~40% margin, confirming an ESP32 anchor performs fine
# under this geometry despite carrying the nRF52-sized budget rather than
# its own conservative default. Do not assume this without measuring on
# a new platform combination -- it happened to fit here, it was not
# derived to fit.
COMMON = {
    "init_count":      N_TAGS,
    "resp_count":      N_ANCHORS,
    "preamble_len":    256,
    "guard_us":        300,
    "resp_base_us":    1364,
    "resp_pitch_us":   1471,
    "report_base_us":  1448,
    "report_pitch_us": 1467,
    "rx_timeout_us":   900,
    "rate_div":        1,
    "report_enable":   1,
    "sync_mmax":       10,

    # Not hashed -- may legitimately differ per board -- but normalized here
    # anyway. $TABLEB checks eq(12)/eq(13) against WHATEVER proc_us/isr_us/
    # margin_us/proc_poll_us/proc_final_us a board currently carries in NVS,
    # regardless of its role: a tag's leftover value from an earlier, more
    # conservative provisioning (e.g. 1500 from an 8-anchor run) can trip
    # TIGHT even though a tag never actually spends this budget. Setting
    # every board to the same values the geometry above was sized for keeps
    # $TABLEB passing everywhere and means any board is ready to serve as an
    # anchor without a second pass if it is ever repurposed.
    "proc_us":         1000,
    "isr_us":          15,
    "margin_us":       100,
    "proc_poll_us":    1000,
    "proc_final_us":   1000,
}

# rx_timeout_us MUST stay well below resp_pitch_us. Measured 2026-09-17: at
# rx_timeout 1200 against resp_pitch 1244, slot 0's RX window stayed open
# until 2150us while Response[1] arrived at 2194us, so slot 0 swallowed slot
# 1's frame and resp_lost doubled to 8.7%.
assert COMMON["rx_timeout_us"] < COMMON["resp_pitch_us"] - 200, "rx_timeout too close to resp_pitch"

EXPECT_FRAME_DUR = 23546
EXPECT_HASH = "0x613D"  # confirmed on A7 (0xC050), 2026-09-21
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
    print(f"{label:>3}  {c.get('node_id')}  {port}")
    if EXPECT_HASH is None:
        print(f"     role={c.get('role')} "
              f"{'frame_index' if role == 'tag' else 'slot_index'}={index} "
              f"T_f={c.get('frame_dur_us')} hash={c.get('_hash')}")
        print(f"     EXPECT_HASH is unset -- this is the reference. "
              f"Set EXPECT_HASH = \"{c.get('_hash')}\" in fleet.py before "
              f"provisioning the next node.")
        return True
    ok = c.get("_hash") == EXPECT_HASH
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

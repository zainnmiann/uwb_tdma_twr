#!/usr/bin/env python3
"""Live raw-range stream, one JSON object per exchange, matching the
tag's own Poll-sent / Response-received timestamps:

  {"tag_label": "T1", "anchor_label": "A2", "range_m": 0.464,
   "rssi": -78.5, "timestamp_sent": 12345678901,
   "timestamp_received": 12345680123}

timestamp_sent     = poll_tx : DW1000 device time the tag transmitted its Poll
timestamp_received = resp_rx : DW1000 device time the tag received that
                     anchor's Response
Both are raw 40-bit DW1000 device-time units (1 unit = 15.650040 ps), taken
directly from the firmware's $TS line (out_mask bit3, RAW_TS) -- enabled on
both tags by this tool at startup. $RNG carries the range/rssi; $TS carries
the timestamps; the firmware emits them back-to-back for the same
(superframe, anchor) pair, which is how they are matched here.

Usage: raw_json.py [tag ...]   (default: both tags)
"""
import sys, os, time, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb import check_line, BOARDS, popen, cmd, sread, get_cfg
import _cleanup

_cleanup.install()

TAGS = sys.argv[1:] or ["ESP_T1", "ESP_T2"]
RAW_TS_BIT = 0x08


def build_labels():
    labels = {}
    for name, (port, baud) in BOARDS.items():
        try:
            s = popen(port, baud, settle=0.3)
        except Exception as e:
            print(f"# {name}: could not open to read labels: {e}", file=sys.stderr)
            continue
        cfg = get_cfg(s)
        s.close()
        node, role = cfg.get("node_id"), cfg.get("role")
        if not node:
            continue
        if role == "INIT" and cfg.get("frame_index") is not None:
            labels[node] = f"T{int(cfg['frame_index']) + 1}"
        elif role == "RESP" and cfg.get("slot_index") is not None:
            labels[node] = f"A{int(cfg['slot_index']) + 1}"
    return labels


_SAVED_MASK = {}


def restore_out_mask():
    """Put each tag's out_mask back as we found it.

    RAW_TS roughly doubles a tag's per-frame serial output, and that
    formatting lands in emit_ranges() right after the Final -- measured to
    push the next Poll's arming past its instant (~12% tx_late). Leaving it
    enabled silently degrades ranging for every other tool, so always
    restore."""
    for name, prev in _SAVED_MASK.items():
        try:
            port, baud = BOARDS[name]
            s = popen(port, baud, settle=0.3)
            cmd(s, f"SET,out_mask,{prev}", attempts=6, window=1.2)
            s.close()
        except Exception as e:
            print(f"# {name}: could not restore out_mask to {prev}: {e}",
                  file=sys.stderr)


def ensure_raw_ts(name):
    """Make sure out_mask has RAW_TS set, without clobbering other bits."""
    port, baud = BOARDS[name]
    s = popen(port, baud, settle=0.3)
    cfg = get_cfg(s)
    try:
        cur = int(cfg.get("out_mask", "3"))
    except ValueError:
        cur = 3
    if name not in _SAVED_MASK:
        _SAVED_MASK[name] = cur
        _cleanup.on_exit(restore_out_mask)
    if not (cur & RAW_TS_BIT):
        cmd(s, f"SET,out_mask,{cur | RAW_TS_BIT}")
    s.close()


def main():
    labels = build_labels()
    for t in TAGS:
        ensure_raw_ts(t)

    ports, bufs = {}, {}
    for t in TAGS:
        p, b = BOARDS[t]
        ports[t] = popen(p, b, settle=0.2)
        _cleanup.on_exit(ports[t].close)
        bufs[t] = b''

    # Pending $RNG record per (tag, sfn, anchor) waiting for its matching $TS.
    pending = {}

    try:
        while True:
            for t, s in ports.items():
                bufs[t] += sread(s, 4096)
                while b'\n' in bufs[t]:
                    raw, bufs[t] = bufs[t].split(b'\n', 1)
                    line = raw.decode('ascii', 'replace').strip()

                    body = check_line(line)
                    if body is None:
                        continue
                    if body.startswith('RNG,'):
                        p = ['$' + body.split(',')[0]] + body.split(',')[1:]
                        if len(p) < 12:
                            continue
                        try:
                            sfn, anchor = p[1], p[4]
                            d_mm = int(p[6])
                            rxp = float(p[8])
                            flags = int(p[10], 16)
                        except ValueError:
                            continue
                        if d_mm <= 0 or (flags & 0x04):   # no valid range / RESP_LOST
                            continue
                        pending[(t, sfn, anchor)] = {
                            "tag_label": labels.get(p[3], p[3]),
                            "anchor_label": labels.get(anchor, anchor),
                            "range_m": round(d_mm / 1000.0, 3),
                            "rssi": rxp,
                        }

                    elif body.startswith('TS,'):
                        p = ['$' + body.split(',')[0]] + body.split(',')[1:]
                        if len(p) < 8:
                            continue
                        sfn, anchor = p[1], p[2]
                        key = (t, sfn, anchor)
                        rec = pending.pop(key, None)
                        if rec is None:
                            continue
                        try:
                            rec["timestamp_sent"] = int(p[3], 16)       # poll_tx
                            rec["timestamp_received"] = int(p[6], 16)   # resp_rx
                        except ValueError:
                            # A board still running firmware built before the
                            # output.c fix for newlib-nano's missing %llX
                            # support (nRF52 targets linked with
                            # --specs=nano.specs) emits literal garbage here
                            # instead of a hex timestamp. Skip the record
                            # rather than crash; reflash to fix at the source.
                            print(f"# {t}: malformed $TS ({p[3]!r},{p[6]!r}) "
                                  f"-- board needs reflashing with the fixed "
                                  f"output.c", file=sys.stderr)
                            continue
                        print(json.dumps(rec), flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        for s in ports.values():
            s.close()


main()

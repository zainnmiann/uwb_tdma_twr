#!/usr/bin/env python3
"""Live ranges as JSON, emitted in true TDMA schedule order.

Same record format as raw_json.py:

  {"tag_label": "T1", "anchor_label": "A1", "range_m": 0.492,
   "rssi": -78.0, "timestamp_sent": 189461991988,
   "timestamp_received": 189569550865}

THREE DIFFERENT CLOCKS ARE IN PLAY -- do not mix them up:

  timestamp_sent      The tag's own microsecond counter (the $RNG line's
                      t_local_us field), stamped on the tag as it builds the
                      record, before any buffering. Wraps every 71.6 min
                      (2^32 us). Use this for update rate and ordering: it
                      is taken at the source, so USB/OS buffering cannot
                      distort it.

  timestamp_received  This host's wall-clock time at decode. Useful only for
                      absolute time-of-day -- USB-CDC delivers bursts of ~50
                      records at once, so many share near-identical values.
                      Never derive rate or ordering from it.

  (the ranging timestamps)
                      Captured in hardware inside the DW1000 at the instant
                      each packet leaves or arrives, in 15.65 ps units, and
                      are what the distance is computed from. They are NOT
                      streamed -- only the resulting tof_ps and range_m are.
                      The firmware can emit them (out_mask bit 3) but that
                      doubles per-frame output and halves the poll rate, so
                      this tool does not enable it.

Ordering: the tags are read over independent USB serial links and USB-CDC
delivers each port in bursts (~16 ms latency timer), so arrival order is
batched per-tag and does NOT match the air schedule. Synchronised tags share
the superframe counter, so records are held briefly and released sorted by
(sfn, frame_index, slot) -- reconstructing what actually happened on air:

    sfn N   frame 0 (T1) -> anchor slot 0, slot 1
    sfn N   frame 1 (T2) -> anchor slot 0, slot 1
    sfn N+1 frame 0 (T1) -> ...

Usage: tdma_order.py [hold_seconds]     (default 2.5)
"""
import sys, os, time, json
from collections import defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb import check_line, BOARDS, popen, sread, get_cfg
import _cleanup

_cleanup.install()

HOLD = float(sys.argv[1]) if len(sys.argv) > 1 else 2.5
TAGS = ["ESP_T1", "ESP_T2"]


def build_labels():
    labels = {}
    for name, (port, baud) in BOARDS.items():
        try:
            s = popen(port, baud, settle=0.3)
        except Exception as e:
            print(f"# {name}: {e}", file=sys.stderr)
            continue
        c = get_cfg(s); s.close()
        node, role = c.get("node_id"), c.get("role")
        if not node:
            continue
        if role == "INIT" and c.get("frame_index") is not None:
            labels[node] = f"T{int(c['frame_index']) + 1}"
        elif role == "RESP" and c.get("slot_index") is not None:
            labels[node] = f"A{int(c['slot_index']) + 1}"
    return labels


def calibrate_sfn_offsets(ports, bufs, seconds=12.0):
    """Each tag keeps its OWN superframe counter. Synchronised tags track
    each other, but the counters settle at a small constant offset, so raw
    sfn is not comparable across tags.

    Rather than guess from arrival times (which are dominated by each USB
    link's own buffering delay and gave a misleading answer), this samples
    real traffic and then sweeps candidate offsets, scoring each by how many
    superframes come out COMPLETE -- i.e. containing exactly the full
    expected set of (frame_index, slot) pairs for the configured network.
    The true offset is the one that reassembles the most whole superframes;
    a wrong offset scatters each superframe's records across two buckets and
    scores visibly lower.
    """
    recs = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        for t, s in ports.items():
            bufs[t] += sread(s, 4096)
            while b'\n' in bufs[t]:
                raw, bufs[t] = bufs[t].split(b'\n', 1)
                l = raw.decode('ascii', 'replace').strip()
                if '$RNG' not in l:
                    continue
                q = l[l.index('$RNG'):].split('*')[0].split(',')
                if len(q) < 12:
                    continue
                try:
                    recs.append((t, int(q[1]), int(q[2]), int(q[5])))
                except ValueError:
                    pass
        time.sleep(0.01)

    frames = sorted({f for _, _, f, _ in recs})
    slots = sorted({s for _, _, _, s in recs})
    want = {(f, s) for f in frames for s in slots}
    ref = TAGS[0]

    offs = {ref: 0}
    for t in TAGS[1:]:
        best_off, best_score = 0, -1
        for off in range(-40, 41):
            buckets = defaultdict(set)
            for tt, sfn, frame, slot in recs:
                if tt not in (ref, t):
                    continue
                buckets[sfn - (off if tt == t else 0)].add((frame, slot))
            score = sum(1 for v in buckets.values() if v == want)
            if score > best_score:
                best_off, best_score = off, score
        offs[t] = best_off
    return offs


def main():
    labels = build_labels()
    ports, bufs = {}, {}
    for t in TAGS:
        p, b = BOARDS[t]
        ports[t] = popen(p, b, settle=0.2)
        _cleanup.on_exit(ports[t].close); bufs[t] = b''

    offs = calibrate_sfn_offsets(ports, bufs)
    print("# sfn offsets (relative to %s): %s" % (TAGS[0], offs), file=sys.stderr)

    pending = {}    # sfn -> [first_seen_ts, [ (frame, slot, record), ... ] ]

    try:
        while True:
            now = time.time()
            for t, s in ports.items():
                bufs[t] += sread(s, 4096)
                while b'\n' in bufs[t]:
                    raw, bufs[t] = bufs[t].split(b'\n', 1)
                    line = raw.decode('ascii', 'replace').strip()

                    now = time.time()
                    body = check_line(line)
                    if body is None or not body.startswith('RNG,'):
                        continue
                    q = body.split(',')
                    if len(q) < 12:
                        continue
                    try:
                        sfn, frame, slot = int(q[1]), int(q[2]), int(q[5])
                        d_mm, flags = int(q[6]), int(q[10], 16)
                        if d_mm <= 0 or (flags & 0x04):
                            continue
                        rec = {
                            "tag_label": labels.get(q[3], q[3]),
                            "anchor_label": labels.get(q[4], q[4]),
                            "range_m": round(d_mm / 1000.0, 3),
                            "rssi": float(q[8]),
                            "timestamp_sent": int(q[11]),     # tag's t_local_us
                            "timestamp_received": round(now, 3),
                        }
                    except ValueError:
                        continue
                    key = sfn - offs.get(t, 0)
                    pending.setdefault(key, [now, []])
                    pending[key][1].append((frame, slot, rec))

            for sfn in sorted(k for k, (ts, _) in pending.items()
                              if now - ts >= HOLD):
                _, items = pending.pop(sfn)
                for _f, _s, rec in sorted(items, key=lambda x: (x[0], x[1])):
                    print(json.dumps(rec), flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        for s in ports.values():
            s.close()


main()

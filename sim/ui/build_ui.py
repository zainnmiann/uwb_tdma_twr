#!/usr/bin/env python3
"""Inline simulator traces into the viewer page.

Usage:  ./build_ui.py <trace-prefix> [out.html]

Reads <prefix>.ideal.json and <prefix>.realistic.json (as written by
`sim --trace=<prefix>`) and substitutes them into template.html. The page is
self-contained afterwards: an artifact cannot fetch anything external, so the
trace has to ship inside it.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def load(prefix, name):
    path = f"{prefix}.{name}.json"
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    prefix = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "viewer.html")

    runs = {}
    for name in ("ideal", "realistic"):
        d = load(prefix, name)
        if d:
            runs[name] = d
    if not runs:
        sys.exit(f"no traces found at {prefix}.*.json -- run sim --trace={prefix}")

    with open(os.path.join(HERE, "template.html")) as f:
        tpl = f.read()

    blob = json.dumps(runs, separators=(",", ":"))
    page = tpl.replace("/*__TRACE_DATA__*/null", blob)

    with open(out, "w") as f:
        f.write(page)

    for name, d in runs.items():
        print(f"  {name:10s} {len(d['events']):5d} events  "
              f"{len(d.get('pairs', [])):3d} pairs  "
              f"pass={d['run']['pass']}")
    print(f"wrote {out} ({os.path.getsize(out) / 1024:.0f} KB)")


if __name__ == "__main__":
    main()

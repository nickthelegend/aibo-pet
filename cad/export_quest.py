"""export_quest.py — write the Quest 3S capture rig to the web.

Companion to export_band.py. Same indexed, normal-free GLB flavour that
web/gl.js reads, same explode-along-one-axis convention so the site can show
it assembled or coming apart with one interpolation.

    .venv/bin/python cad/export_quest.py
"""
from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import part_quest as Q
import partlib as pl
from export_web import write_glb

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, "..", "thenar"))

EXPLODE = {
    "quest-shell": (0, 0, 0), "quest-strap": (0, 40, 30),
    "quest-front": (0, -70, 0), "quest-lens": (0, -140, 0),
    "quest-pod": (0, -105, 0),
    "quest-face": (0, 78, 0),
    "ctrl-l-shell": (-70, 0, -30), "ctrl-l-dark": (-105, 0, -30),
    "ctrl-r-shell": (70, 0, -30), "ctrl-r-dark": (105, 0, -30),
}


def main():
    parts = Q.assembled()
    items, meta = [], []
    for name, m in parts.items():
        rep = pl.validate(m)
        if not rep["watertight"]:
            print(f"FAIL {name}: {rep['problems'][:3]}")
            return 1
        items.append((name, m.V, m.F, Q.COLOURS[name]))
        b = m.bounds()
        meta.append({"name": name, "colour": Q.COLOURS[name],
                     "explode": EXPLODE[name],
                     "size_mm": [round(b[i + 3] - b[i], 1) for i in range(3)],
                     "triangles": rep["triangles"]})

    glb = os.path.join(OUT, "quest.glb")
    write_glb(glb, items)
    with open(os.path.join(OUT, "quest.json"), "w") as f:
        json.dump({"parts": meta,
                   "note": "Our own likeness of hardware we capture on. "
                           "No Meta marks; dimensions approximate."}, f, indent=1)
    print(f"quest.glb  {os.path.getsize(glb)/1024:.0f} kB, {len(items)} parts, "
          f"{sum(x['triangles'] for x in meta)} triangles")
    return 0


if __name__ == "__main__":
    sys.exit(main())

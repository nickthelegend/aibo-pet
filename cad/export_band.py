"""export_band.py — write the Thenar Band to the web and to STLs.

Two poses go into one GLB, both in world coordinates:

    assembled   worn: cap seated, camera twisted home in the bayonet
    exploded    the same parts pushed out along the axis they come apart on

The site interpolates between them, so one 200 kB file is both the product
shot and the teardown. The GLB is the indexed, normal-free flavour that
web/gl.js already reads, and the shader rebuilds flat normals from
derivatives, so shipping NORMAL would be dead weight.

    .venv/bin/python cad/export_band.py
"""
from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import part_band as B
import partlib as pl
from export_web import write_glb

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
OUT = os.path.join(ROOT, "thenar")
STL = os.path.join(ROOT, "exports", "band")

# How each part leaves the assembly. The cap lifts off the bay, the camera
# withdraws along its own mount axis, the pad swings away from the palm.
EXPLODE = {
    "band-cuff": (0.0, 0.0, 0.0),
    "band-cap":  (0.0, 42.0, 0.0),
    "band-cam":  (0.0, 26.0, 62.0),
    "band-pad":  (0.0, -46.0, 0.0),
}

ROLE = {
    "band-cuff": "Wrist chassis. Springs over the wrist; the strap takes up the rest of the range.",
    "band-cap":  "Removable bay lid. Lip drops into the mouth, thumb notch levers it off with a glove on.",
    "band-cam":  "Detachable camera. Three lugs, drop in and twist 25 degrees to a hard stop.",
    "band-pad":  "Thenar pressure pad on a compliant arm, so it stays loaded as the thumb opposes.",
}


def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(STL, exist_ok=True)

    parts = B.assembled()

    items, meta = [], []
    for name, m in parts.items():
        rep = pl.validate(m)
        if not rep["watertight"]:
            print(f"FAIL {name}: {rep['problems'][:3]}")
            return 1
        items.append((name, m.V, m.F, B.COLOURS[name]))
        b = m.bounds()
        meta.append({
            "name": name,
            "colour": B.COLOURS[name],
            "role": ROLE[name],
            "explode": EXPLODE[name],
            "size_mm": [round(b[i + 3] - b[i], 1) for i in range(3)],
            "volume_mm3": round(rep["volume_mm3"], 1),
            "triangles": rep["triangles"],
        })

    glb = os.path.join(OUT, "band.glb")
    write_glb(glb, items)

    # print pose STLs, from the unassembled builders
    for name, m in B.build().items():
        pl.stl_write(os.path.join(STL, name + ".stl"), m)

    with open(os.path.join(OUT, "band.json"), "w") as f:
        json.dump({"parts": meta,
                   "mount": {"tilt_deg": B.MOUNT_TILT, "twist_deg": B.BAYO_TWIST,
                             "lugs": 3},
                   "wrist_bore_mm": [B.WRIST_X, B.WRIST_Y]}, f, indent=1)

    kb = os.path.getsize(glb) / 1024
    print(f"band.glb   {kb:.0f} kB, {len(items)} parts, "
          f"{sum(x['triangles'] for x in meta)} triangles")
    for x in meta:
        print(f"  {x['name']:11s} {str(x['size_mm']):22s} {x['volume_mm3']:9.0f} mm3")
    return 0


if __name__ == "__main__":
    sys.exit(main())

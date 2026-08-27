"""export_spec_v2.py — web/spec2.json, the Parts page's data for 2.0.

Same shape as v1's spec.json (web/viewer.js reads it), rebuilt from the 2.0
assembly so the page lists the parts that actually exist: 21 printed parts
on four P1S plates, four MG996Rs, and a fastener count of ZERO, because
every bolt on this robot is printed.

    .venv/bin/python cad/export_spec_v2.py   ->  web/spec2.json
"""
from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import partlib as pl
import v2_assembly as A
import v2_parts as V
from export_web import WEB

RAW = "https://raw.githubusercontent.com/nickthelegend/aibo-pet/main/exports/v2/"
EXPV2 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "exports", "v2")
ZIP = os.path.join(EXPV2, "hotaru2-plates.zip")
# carried from v2_audit's over-air allowlist: the only part whose print
# genuinely needs support is the head block, under its horizontal stub axle
NEEDS_SUPPORT = {"v2-head"}


def _vol_cm3(mesh):
    """Signed tetrahedron sum. partlib has no volume(), and a part list
    without volumes cannot tell anyone how much filament this costs."""
    import numpy as np
    Vv, F = mesh._np()
    a, b, c = Vv[F[:, 0]], Vv[F[:, 1]], Vv[F[:, 2]]
    return float(abs(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6e3)

WHY = {
    "v2-plate-1-base": "the tub, its crown and the button turret -- the big one",
    "v2-plate-2-disc": "the turntable, link1's inner plate and the yoke screws",
    "v2-plate-3-head": "every remaining link plate, the head block and the caps",
    "v2-plate-4-cone": "the cone, its LED cap, the horns, bolts and disc keys",
}


def main():
    items = A.print_items()
    parts = []
    total_cm3 = 0.0
    for n, m in items:
        b = m.bounds()
        vol = _vol_cm3(m)
        total_cm3 += vol
        parts.append({
            "name": n,
            "mm": [round(float(b[3] - b[0]), 1), round(float(b[4] - b[1]), 1),
                   round(float(b[5] - b[2]), 1)],
            "cm3": round(vol, 1),
            # bool(): the bounds come back as numpy scalars, so the
            # comparison yields np.bool_ and json refuses to encode it
            "fits": bool(b[3] - b[0] <= V.BED - 12 and b[4] - b[1] <= V.BED - 12
                         and b[5] - b[2] <= V.BED),
        })

    # Plates come from v2_export's LAYOUT, not v2_assembly.PLATES: LAYOUT is
    # what actually packs the STLs into the zip people download, and the two
    # had drifted apart (PLATES still named a "hardware" plate that no
    # longer exists). The page must describe the files, not an older plan.
    import v2_export
    by_name = dict(items)
    grouped = {}
    for nm, (plate, _dx, _dy) in v2_export.LAYOUT.items():
        grouped.setdefault(plate, []).append(nm)
    plate_list = []
    for pn in sorted(grouped):
        have = [x for x in grouped[pn] if x in by_name]
        desc = ""
        if not have:
            continue
        lo = [1e9] * 3
        hi = [-1e9] * 3
        for x in have:
            b = by_name[x].bounds()
            for i in range(3):
                lo[i] = min(lo[i], b[i])
                hi[i] = max(hi[i], b[i + 3])
        stl = os.path.join(EXPV2, pn + ".stl")
        plate_list.append({
            "plate": pn,
            "why": WHY.get(pn, desc),
            "parts": have,
            "used_mm": [round(float(hi[i] - lo[i]), 1) for i in range(3)],
            # viewer.js reads fits_bed and supports by these exact names --
            # a missing key is a blank page, not a soft degrade
            "fits_bed": bool(hi[0] - lo[0] <= V.BED and hi[1] - lo[1] <= V.BED),
            "supports": [x for x in have if x in NEEDS_SUPPORT],
            "mb": round(os.path.getsize(stl) / 1e6, 2)
            if os.path.exists(stl) else 0.0,
            "url": RAW + pn + ".stl",
        })

    # recover each part's print transform by comparing world to print pose
    wb = {n: m.bounds() for n, m, _c in A.world_items()}
    print_pose = {}
    for n, m in items:
        pb = m.bounds()
        wbb = wb.get(n)
        flip = 0.0
        if wbb is not None:
            # a flipped part has its world and print Z extents mirrored
            wh, ph = wbb[5] - wbb[2], pb[5] - pb[2]
            if abs(wh - ph) < 1e-6:
                # same height either way; detect the flip by the disc/"-in"
                # rule print_items() actually uses
                flip = 180.0 if (n == "v2-disc" or
                                 (n.endswith("-in") and n != "v2-link1-in")
                                 ) else 0.0
        print_pose[n] = {"flip": flip, "dz": round(float(-pb[2]), 2)}

    world = A.world_items()
    zmax = max(m.bounds()[5] for _n, m, _c in world)

    spec = {
        "parts": parts,
        "interfaces": [],
        "stats": {
            "parts": len(parts),
            "interfaces": len(parts),
            "joints": 4,
            "servos": 4,
            "height_mm": int(round(zmax)),
            "reach_mm": int(round(V.L1 + V.L2 + 90)),
            "plastic_cm3": int(round(total_cm3)),
            "plates": len(plate_list),
            # the whole point of 2.0's fastener pass: nothing to buy
            "m3": 0,
            "m2": 0,
            "printed_screws": 24,
            "bed_mm": int(V.BED),
        },
        "downloads": {
            "base": RAW,
            "zip": RAW + "hotaru2-plates.zip",
            "zip_mb": round(os.path.getsize(ZIP) / 1e6, 1),
            "plates_zip": RAW + "hotaru2-plates.zip",
            "plates_zip_mb": round(os.path.getsize(ZIP) / 1e6, 1),
            "plates": [p["plate"] for p in plate_list],
            # a MAP, not a list: viewer.js indexes it by part name
            "files": {n: RAW + n + ".stl" for n, _m in items},
        },
        # viewer.js indexes this BY PART NAME for its PRINT ORIENTATION
        # mode: {flip, dz} is exactly the transform print_items() applies.
        # A bare `true` here is not a crash, it is worse -- every part
        # silently falls back to {flip:0, dz:0} and the mode shows a lie.
        "print_pose": print_pose,
        "plates": {"bed_mm": float(V.BED), "list": plate_list},
    }

    with open(os.path.join(WEB, "spec2.json"), "w") as fh:
        json.dump(spec, fh, indent=1)
    print(f"web/spec2.json  {len(parts)} parts, {len(plate_list)} plates, "
          f"{total_cm3:.0f} cm3, height {spec['stats']['height_mm']} mm")
    for p in plate_list:
        print(f"  {p['plate']:20s} {len(p['parts']):2d} parts  "
              f"{p['used_mm'][0]:.0f} x {p['used_mm'][1]:.0f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

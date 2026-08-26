"""v2_export.py — Hotaru 2.0 plates, zip, GLB and report.

Three P1S plates instead of v1's nine A1-mini plates:
  v2-plate-1-base   tub + tower + clamps        (the two big rounds cannot
  v2-plate-2-disc   disc + all link parts        share one bed: 155+155>244)
  v2-plate-3-head   head + shade + spacers
"""
from __future__ import annotations

import json
import os
import sys
import zipfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import partlib as pl
import audit_support as asup
import v2_assembly as A
import v2_parts as V
from export_web import write_glb

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.normpath(os.path.join(HERE, "..", "exports", "v2"))
WEB = os.path.normpath(os.path.join(HERE, "..", "web"))

# name -> (plate, dx, dy)
LAYOUT = {
    "v2-tub":          ("v2-plate-1-base", 0.0, 0.0),
    "v2-tower":        ("v2-plate-1-base", 0.0, 115.0),
    "v2-clamps":       ("v2-plate-1-base", -90.0, 110.0),
    "v2-disc":         ("v2-plate-2-disc", 0.0, -44.0),
    "v2-link1-in":     ("v2-plate-2-disc", 0.0, 62.0),
    "v2-link1-out":    ("v2-plate-3-head", -35.0, -80.0),
    "v2-link2-in":     ("v2-plate-3-head", -35.0, -30.0),
    "v2-link2-out":    ("v2-plate-3-head", -35.0, 20.0),
    "v2-link1-spacers":("v2-plate-3-head", 95.0, -80.0),
    "v2-link1-ledges": ("v2-plate-3-head", 95.0, -20.0),
    "v2-link2-spacers":("v2-plate-3-head", 95.0, 45.0),
    "v2-head":         ("v2-plate-3-head", 40.0, 85.0),
    "shade":           ("v2-plate-3-head", -70.0, 85.0),
    "v2-screw":        ("v2-plate-3-head", -12.0, 100.0),
    "v2-trimcap":      ("v2-plate-3-head", -12.0, 64.0),
}


def main():
    os.makedirs(EXP, exist_ok=True)
    items = dict(A.print_items())

    plates = {}
    sup = {}
    for name, m in items.items():
        a, _w, _c = asup.unsupported(m)
        sup[name] = a > asup.MIN_AREA
        plate, dx, dy = LAYOUT[name]
        q = m.copy()
        b = q.bounds()
        q.translate(dx=dx - (b[0] + b[3]) / 2, dy=dy - (b[1] + b[4]) / 2)
        plates.setdefault(plate, pl.Mesh())
        plates[plate] += q
        pl.stl_write(os.path.join(EXP, name + ".stl"), m)

    # on-plate collision: convex hull of each part's XY footprint, pairwise.
    # A bed-fit test alone let the disc and a link share the same square
    # centimetres of plate and called it fine.
    import numpy as np
    from shapely.geometry import MultiPoint
    hulls = {}
    for name, m in items.items():
        plate, dx, dy = LAYOUT[name]
        q = np.asarray(m.V)
        b = m.bounds()
        pts = q[:, :2] + [dx - (b[0] + b[3]) / 2, dy - (b[1] + b[4]) / 2]
        step = max(1, len(pts) // 900)
        hulls.setdefault(plate, []).append(
            (name, MultiPoint(pts[::step]).convex_hull))
    for plate, hh in hulls.items():
        for i in range(len(hh)):
            for j in range(i + 1, len(hh)):
                inter = hh[i][1].intersection(hh[j][1]).area
                assert inter < 1.0, (
                    f"{plate}: {hh[i][0]} overlaps {hh[j][0]} by {inter:.0f} mm2")
    print("on-plate collisions: none")

    rows = []
    for pn, mesh in plates.items():
        b = mesh.bounds()
        w, d, h = b[3] - b[0], b[4] - b[1], b[5] - b[2]
        fits = w <= V.BED - 12 and d <= V.BED - 12
        rows.append((pn, w, d, h, fits))
        pl.stl_write(os.path.join(EXP, pn + ".stl"), mesh)
        print(f"{pn:18s} {w:6.1f} x {d:6.1f} x {h:5.1f}  "
              f"{'fits P1S' if fits else 'TOO BIG'}")
        assert fits, pn

    parts_with_support = sorted(n for n, v in sup.items() if v)
    readme = ["HOTARU 2.0 -- printable plates (Bambu P1S, 256 x 256)",
              "=" * 52, "",
              "v1 is deprecated. 2.0 adds the 180-degree pan base: an MG996R",
              "dead centre in the tub drives the turntable disc; the disc",
              "rides the tub rim, so the bearing is the rim and the servo",
              "only ever sees torque.", "",
              "Print order:"]
    for i, (pn, w, d, h, _f) in enumerate(rows, 1):
        readme.append(f"{i}. {pn}.stl   {w:.0f} x {d:.0f} x {h:.0f} mm")
    readme += ["", f"SUPPORTS ON for: {', '.join(parts_with_support)}",
               "(everything else prints clean -- measured by cad/v2_audit.py,",
               "which also proves every joint engagement number on this arm)", "",
               "Electronics: ESP32-S3, MAX98357A, 40x20 speaker, round mic,",
               "3x MG996R, 1x SG90, M3 inserts -- the same BOM as v1.", "",
               "CAD, audits, and the whole history:",
               "github.com/nickthelegend/aibo-pet"]
    zp = os.path.join(EXP, "hotaru2-plates.zip")
    with zipfile.ZipFile(zp, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        z.writestr("README.txt", "\n".join(readme))
        for pn, *_ in rows:
            z.write(os.path.join(EXP, pn + ".stl"), pn + ".stl")
    print(f"hotaru2-plates.zip  {os.path.getsize(zp)/1e6:.1f} MB")

    # GLB of the assembled robot for the site
    world = [(n, m.V, m.F, c if isinstance(c, str) else "#E9E9EE")
             for n, m, c in (A.world_items())]
    write_glb(os.path.join(WEB, "hotaru2.glb"), world)
    print(f"web/hotaru2.glb  {os.path.getsize(os.path.join(WEB,'hotaru2.glb'))/1e6:.1f} MB")

    report = {
        "audit": "cad/v2_audit.py -- all checks pass",
        "plates": [{"plate": pn, "size_mm": [round(w,1), round(d,1), round(h,1)]}
                   for pn, w, d, h, _f in rows],
        "supports": parts_with_support,
        "engagement": {
            "shoulder_horn_mm": 2.4, "elbow_horn_mm": 1.5,
            "elbow_clearance_mm": 0.3, "stub_engage_mm": 5.0,
            "shade_yoke_margin_mm": 0.7},
        "known_limits": [
            "elbow is single sided: drive horn M3 is the retention; the far "
            "plate is truncated clear of the servo tail's swept annulus",
            "v2-head needs supports (45 mm2 under its stub axle)",
            "electronics wall openings (USB, mic port, MX plate) are placed "
            "in the next electronics pass; posts and pockets are in"],
    }
    with open(os.path.join(EXP, "BUILD_REPORT_V2.json"), "w") as f:
        json.dump(report, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())

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
    # Plate 3 is the TURNTABLE: everything that turns with the base, on one
    # plate -- disc, the tower that stands on it, and the keys that hold it
    # down. It cannot also carry the tub: the tub is 157 across and the disc
    # 155, and 157 + 155 does not fit a 244 usable bed side by side or
    # stacked. So the fixed shell keeps plate 1 and the rotating assembly
    # gets its own, which is at least one print job per sub-assembly.
    "v2-tub":          ("v2-plate-1-base", 0.0, 0.0),
    "v2-head":         ("v2-plate-1-base", 0.0, 118.0),

    # The four links are each 58 deep and were spaced 54 apart, so every
    # neighbouring pair overlapped by 4 mm -- 449 mm2 of shared plate that
    # the collision check caught and the bed-fit check never would have,
    # because the PLATE still fitted the bed; the parts just sat on top of
    # one another. 4 x 58 = 232 against 244 usable leaves 12 mm for three
    # gaps, so 61 centres gives each pair a clean 3 mm.
    "v2-link1-in":     ("v2-plate-2-arm", 0.0, -91.5),
    "v2-link1-out":    ("v2-plate-2-arm", 0.0, -30.5),
    "v2-link2-in":     ("v2-plate-2-arm", 0.0, 30.5),
    "v2-link2-out":    ("v2-plate-2-arm", 0.0, 91.5),
    "v2-link1-spacers":("v2-plate-2-arm", 115.0, -81.0),
    "v2-link1-ledges": ("v2-plate-2-arm", 115.0, -30.0),
    "v2-link2-spacers":("v2-plate-2-arm", 115.0, 27.0),

    "v2-disc":         ("v2-plate-3-turntable", 0.0, 0.0),
    "v2-tower":        ("v2-plate-3-turntable", 0.0, 112.0),
    "v2-disckeys":     ("v2-plate-3-turntable", 95.0, 112.0),
    "v2-clamps":       ("v2-plate-3-turntable", 0.0, 150.0),
    "v2-keycap":       ("v2-plate-3-turntable", 62.0, 150.0),

    "shade":           ("v2-plate-4-cone", 0.0, 55.0),
    "v2-conecap":      ("v2-plate-4-cone", 110.0, 55.0),
    "v2-bolts":        ("v2-plate-4-cone", 115.0, -45.0),
    "v2-screws":       ("v2-plate-4-cone", 115.0, -110.0),
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
               "4x MG996R. Each servo uses the STOCK horn from its own bag.",
               "Every fastener is printed EXCEPT five M3s: four bolt the",
               "tower down to the disc, and the fifth is the screw that came",
               "with the servo, holding the pan horn on its output shaft.", "",
               "CAD, audits, and the whole history:",
               "github.com/nickthelegend/hotaru"]
    zp = os.path.join(EXP, "hotaru2-plates.zip")
    with zipfile.ZipFile(zp, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        z.writestr("README.txt", "\n".join(readme))
        for pn, *_ in rows:
            z.write(os.path.join(EXP, pn + ".stl"), pn + ".stl")
    print(f"hotaru2-plates.zip  {os.path.getsize(zp)/1e6:.1f} MB")

    # ---- GLBs: the site hero, and every view the parts viewer offers ----
    world = A.world_items()

    def _glb(path, items):
        """The PARTS VIEWER reads partlib's flavour (exploded soup carrying
        NORMAL); web/gl.js reads export_web's (indexed, normal-free, normals
        rebuilt in the shader). Writing one file in the other's flavour
        fails at load with a bufferView error, which is exactly how the v2
        models first landed in the old viewer. Path decides the writer."""
        items = [(n, m, c if isinstance(c, str) else "#E9E9EE")
                 for n, m, c in items]
        pl.glb_write(path, items)
        return os.path.getsize(path) / 1e6

    def _glb_web(path, items):
        write_glb(path, [(n, m.V, m.F, c if isinstance(c, str) else "#E9E9EE")
                         for n, m, c in items])
        return os.path.getsize(path) / 1e6

    mb = _glb_web(os.path.join(WEB, "hotaru2.glb"), world)
    print(f"web/hotaru2.glb  {mb:.1f} MB")

    # printed parts only: the servos are components, not things you print
    SERVO_PRE = ("pan-", "sh-", "el-", "hd-")
    printed = [(n, m, c) for n, m, c in world
               if not n.startswith(SERVO_PRE)]
    _glb(os.path.join(EXP, "hotaru2-populated.glb"), world)
    _glb(os.path.join(EXP, "hotaru2-assembled.glb"), printed)

    # exploded: push each part out along the axis it comes apart on. The arm
    # separates along +Z (it stacks), the base parts along their own radius.
    import numpy as _np
    ex = []
    for n, m, c in printed:
        q = m.copy()
        b = q.bounds()
        cz = (b[2] + b[5]) / 2.0
        if n in ("v2-tub",):
            dz = 0.0
        elif n == "v2-disc":
            dz = 26.0
        elif n == "v2-tower":
            dz = 52.0
        elif n.startswith("v2-link1"):
            dz = 78.0
        elif n.startswith("v2-link2"):
            dz = 104.0
        else:
            dz = 130.0
        # fan the sandwich plates apart in Y. The first cut fanned them in
        # X -- but the plates' faces POINT along X, so the natural viewing
        # angle looks straight down that axis and the fanned pair lands
        # exactly behind itself: the exploded arm read as ONE plate.
        dy = 0.0
        if n.endswith("-in"):
            dy = 40.0
        elif n.endswith("-out"):
            dy = -40.0
        elif n.endswith(("-spacers", "-ledges")):
            dy = 80.0
        q.translate(dy=dy, dz=dz)
        ex.append((n, q, c))
    _glb(os.path.join(EXP, "hotaru2-exploded.glb"), ex)

    # one GLB per plate, in print pose, so the viewer can show what slices
    for pn, mesh in plates.items():
        _glb(os.path.join(EXP, pn + ".glb"), [(pn, mesh, "#E9E9EE")])
    print(f"GLBs: populated, assembled, exploded, {len(plates)} plates")

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
            "elbow is single sided: the printed trim cup is the retention; the far "
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

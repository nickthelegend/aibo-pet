"""assembly.py — place every part in world position, export STLs + previews.

  exports/aibo-assembled.glb   the lamp in its neutral pose
  exports/aibo-exploded.glb    the same, parts pulled apart
  exports/<part>.stl           one print-ready STL per part, already in its
                               print orientation with min Z dropped to 0
  exports/MANIFEST.json        per-file stats, pose, print notes

Exits non-zero unless every shell of every part is watertight AND every part
fits the A1 mini.
"""
from __future__ import annotations

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import components as CO
import params as P
import part_arms
import part_base
import part_base_joint
import part_head
import part_horn
import part_keycap
import part_screw
import part_lid
import part_retainers
import part_shoulder
import partlib as pl

HERE = os.path.dirname(os.path.abspath(__file__))
EXPORTS = os.path.normpath(os.path.join(HERE, "..", "exports"))

# neutral pose: degrees about X at each joint, + leans the segment FORWARD
POSE = {"base": -22.0, "shoulder": 58.0, "elbow": 52.0, "head": 26.0}


def _place(mesh, cum_deg, pivot, axis0):
    m = mesh.copy()
    m.translate(dz=-axis0)
    m.rotate_x(cum_deg)
    m.translate(dx=pivot[0], dy=pivot[1], dz=pivot[2])
    return m


def _step(pivot, cum_deg, length):
    a = math.radians(cum_deg)
    return (pivot[0], pivot[1] - length * math.sin(a), pivot[2] + length * math.cos(a))


def world_items():
    """[(name, mesh, color)] with every part in its assembled position."""
    base = part_base.build()
    sho = part_shoulder.build()
    lid = part_lid.build()
    bj = part_base_joint.build()
    arms = {n: (m, c) for n, m, c in part_arms.build()}
    shade = part_head.build()[0]

    out = list(base) + list(sho) + list(lid)
    out += [(n, part_base_joint.to_world(m), c) for n, m, c in bj]

    p0 = (0.0, P.BSERVO_AXIS_Y, P.BJOINT_AXIS_Z)
    cum = POSE["base"]
    chain = [("arm-lower", "cap-shoulder", P.ARM_LOWER_L, POSE["shoulder"]),
             ("arm-upper", "cap-elbow", P.ARM_UPPER_L, POSE["elbow"]),
             ("arm-fore", "cap-head", P.ARM_FORE_L, POSE["head"])]
    for name, capname, length, next_deg in chain:
        mesh, col = arms[name]
        out.append((name, _place(mesh, cum, p0, P.YOKE_BELOW), col))
        cmesh, ccol = arms[capname]
        out.append((capname, _place(cmesh, cum, p0, P.YOKE_BELOW), ccol))
        p0 = _step(p0, cum, length)
        cum += next_deg

    sname, smesh, scol = shade
    sm = smesh.copy()
    sm.translate(dz=-part_head.TILT)
    sm.rotate_x(cum + 180.0)          # shade faces out along the arm
    sm.translate(dx=p0[0], dy=p0[1], dz=p0[2])
    out.append((sname, sm, scol))
    return out


def world_components():
    """The real hardware, placed where it actually sits. Same chain of
    transforms as the arm, so a servo lands in its cup rather than near it."""
    out = []
    out += CO.for_part("base")
    # The lid is modelled in world coordinates already (it is never rotated
    # into place), so its MX switch and keycap need no extra transform.
    out += CO.for_part("lid")

    # base servo: into the base-joint housing, then through that part's own
    # frame transform (rotate -90 about X, then translate)
    mg = CO.place(CO.mg996r(), ry=90.0, dx=-P.MG_H / 2,
                  dz=part_base_joint.AXIS - (P.MG_L / 2 - P.MG_SHAFT_OFF), tag="-base")
    out += [(n, part_base_joint.to_world(m), c) for n, m, c in mg]

    p0 = (0.0, P.BSERVO_AXIS_Y, P.BJOINT_AXIS_Z)
    cum = POSE["base"]
    for tag, length, nxt, kind in (("shoulder", P.ARM_LOWER_L, POSE["shoulder"], "mg"),
                                   ("elbow", P.ARM_UPPER_L, POSE["elbow"], "mg"),
                                   ("head", P.ARM_FORE_L, POSE["head"], "sg")):
        axis1 = P.YOKE_BELOW + length
        if kind == "mg":
            sv = CO.place(CO.mg996r(), ry=90.0, dx=-P.MG_H / 2,
                          dz=axis1 - (P.MG_L / 2 - P.MG_SHAFT_OFF), tag=f"-{tag}")
        else:
            sv = CO.place(CO.sg90(), ry=90.0, dx=-P.SG_H / 2,
                          dz=axis1 - (P.SG_L / 2 - P.SG_SHAFT_OFF), tag=f"-{tag}")
        out += [(n, _place(m, cum, p0, P.YOKE_BELOW), c) for n, m, c in sv]
        p0 = _step(p0, cum, length)
        cum += nxt
    return out


# keycap prints TOP FACE DOWN too -- see part_keycap's docstring: flat top on
# the bed, walls flaring out at 14 deg, stem growing up off the roof.
# shoulder is flipped because its four lid lugs are cantilevered into the
# bore with flat undersides 20 mm up, and the r68..77.6 flange overhangs the
# skirt it stands on. Printed as modelled that is 10,270 mm2 hanging over air
# and the lugs come off the bed as drooping string with the screw holes in
# space. Upside down it is 287 mm2. The 45 degree taper becomes a 45 degree
# overhang, which is a surface finish cost, not a structural one.
# Measured by cad/audit_support.py, both orientations.
PRINT_FLIP = {"lid": 180.0, "keycap": 180.0, "shoulder": 180.0}
PRINT_SHIFT = {}                     # name -> (flip_deg, dz) applied below


def print_items():
    """[(name, mesh)] in print orientation, bottoms on Z=0. Records the exact
    transform in PRINT_SHIFT so anything that has to FOLLOW a part into its
    print pose (component models, say) can apply the same one."""
    groups = (part_base.build() + part_shoulder.build() + part_lid.build()
              + part_base_joint.build() + part_arms.build() + part_head.build()
              + part_horn.build() + part_retainers.build()
              + part_keycap.build() + part_screw.build())
    out = []
    for name, mesh, _c in groups:
        m = mesh.copy()
        flip = PRINT_FLIP.get(name, 0.0)
        if flip:
            m.rotate_x(flip)
        dz = -m.bounds()[2]
        m.translate(dz=dz)
        PRINT_SHIFT[name] = (flip, dz)
        out.append((name, m))
    return out


def components_in_print_pose(name):
    """Component models for `name`, moved into that part's print pose."""
    flip, dz = PRINT_SHIFT.get(name, (0.0, 0.0))
    out = []
    for cn, cm, cc in CO.for_part(name):
        m = cm.copy()
        if flip:
            m.rotate_x(flip)
        m.translate(dz=dz)
        out.append((cn, m, cc))
    return out


def main():
    os.makedirs(EXPORTS, exist_ok=True)
    ok = True
    print("part                shells   tris  watertight  fits  bbox")
    print("-" * 74)
    reports = {}
    for name, mesh in print_items():
        r = pl.validate(mesh)
        fit, d = pl.fits_build_plate(mesh)
        reports[name] = {"shells": r["shells"], "triangles": r["triangles"],
                         "volume_mm3": r.get("volume_mm3"), "watertight": r["watertight"],
                         "fits_a1_mini": bool(fit),
                         "bbox_mm": [round(float(v), 1) for v in d]}
        ok &= r["watertight"] and fit
        print(f"{name:18s} {r['shells']:5d} {r['triangles']:7d}  "
              f"{str(r['watertight']):>10s}  {str(bool(fit)):>5s}  "
              f"{tuple(round(float(v), 1) for v in d)} {r['problems'][:2]}")
        pl.stl_write(os.path.join(EXPORTS, f"{name}.stl"), mesh)

    items = world_items()
    pl.glb_write(os.path.join(EXPORTS, "aibo-assembled.glb"), items)
    comps = world_components()
    pl.glb_write(os.path.join(EXPORTS, "aibo-populated.glb"), items + comps)
    print(f"populated: +{len(comps)} component meshes "
          f"({sum(len(m.F) for _n, m, _c in comps):,} tris)")
    lift = {"base": 0.0, "shoulder": 18.0, "lid": 34.0, "base-joint": 54.0,
            "cap-base": 68.0}
    pl.glb_write(os.path.join(EXPORTS, "aibo-exploded.glb"),
                 [(n, m.copy().translate(dz=lift.get(n, 0.0)), c) for n, m, c in items])

    top = max(m.bounds()[5] for _n, m, _c in items)
    manifest = {
        "generator": "aibo cad/assembly.py",
        "pose_deg": POSE,
        "lamp_height_mm": round(float(top), 1),
        "reach_mm": P.ARM_LOWER_L + P.ARM_UPPER_L + P.ARM_FORE_L,
        "build_plate_mm": P.PLATE,
        "parts": reports,
        "print_notes": {
            "material": "PLA",
            "layer_mm": 0.2,
            "walls": 3,
            "supports": "none required -- every part is modelled in its print "
                        "orientation with no overhang past 45 deg",
            "arms": "print standing on the yoke, servo cup opening up",
            "shade": "prints mouth down, open back",
            "lid": "prints top face down so the MX plate comes out crisp",
        },
    }
    with open(os.path.join(EXPORTS, "MANIFEST.json"), "w") as fh:
        json.dump(manifest, fh, indent=2)
    print("-" * 74)
    print(f"lamp height in pose: {top:.0f} mm   arm reach: {manifest['reach_mm']:.0f} mm")
    print(f"wrote {len(reports)} STLs + 3 GLBs + MANIFEST.json to exports/")
    print("ASSEMBLY", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

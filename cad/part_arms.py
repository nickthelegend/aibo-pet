"""part_arms.py — the three arm segments + their servo caps.

Each segment is one printable part carrying, bottom to top:
  yoke      the fork that straddles the PREVIOUS joint's housing -- drive
            plate (+X, cross recess onto the servo horn) and idler plate
            (-X, running on the housing's stub axle)
  converge  the two plates neck inward and merge into the tube. 28 mm for
            25.45 mm of inward travel = 42 deg, self-supporting.
  tube      closed hollow section, ARM_WALL walls, with exterior zip-tie
            clips on the back face for the service loop
  flare     tube -> housing at 38 deg
  housing   the NEXT joint's servo cup, open at the top

Printed exactly as modelled: standing on the yoke, cup opening up. The two
yoke plates start as separate islands on the plate and merge as the print
rises. No bridges, no overhangs past 45 deg, no supports.
"""
from __future__ import annotations

import os
import sys

import numpy as np
from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import joints as J
import params as P
import partlib as pl

OVL = pl.OVL


def converge(z0, z1, steps=28):
    """Two yoke plates neck inward and merge into the arm tube."""
    zs = np.linspace(z0, z1, steps + 1)
    ax, ay = P.ARM_W / 2.0, P.ARM_D / 2.0
    m = pl.Mesh()
    for i in range(steps):
        t = (i + 0.5) / steps
        xi = J.YK_X0 * (1 - t)
        xo = J.YK_X1 * (1 - t) + ax * t
        yy = (P.YOKE_DEPTH / 2.0) * (1 - t) + ay * t
        if xi <= 0.3:
            prof = box(-xo, -yy, xo, yy)
        else:
            prof = unary_union([box(xi, -yy, xo, yy), box(-xo, -yy, -xi, yy)])
        m += pl.prism(prof, zs[i] - (OVL if i else 0),
                      zs[i + 1] + (OVL if i < steps - 1 else 0))
    return m


def segment(length, top="mg"):
    """One arm segment. Returns (mesh, cap_mesh, axis0, axis1)."""
    axis0 = P.YOKE_BELOW
    axis1 = axis0 + length
    m = pl.Mesh()
    m += J.yoke(axis0)

    z_cv0 = axis0 + P.YOKE_ABOVE
    z_cv1 = z_cv0 + P.CONVERGE_Z
    m += converge(z_cv0, z_cv1)

    if top == "mg":
        hz_bot = axis1 - (P.MG_L - P.MG_SHAFT_OFF) - P.JOINT_WALL
        flare_len = P.FLARE_Z
        house, hz_bot, hz_top = J.housing(axis1)
        cap = J.housing_cap(axis1, hz_top)
        outer = pl.rounded_rect(2 * J.H_HX, 2 * J.H_HY, 3.0)
    else:
        house, hz_bot, hz_top = sg_housing(axis1)
        flare_len = 10.0
        cap = sg_cap(hz_top)
        outer = pl.rounded_rect(2 * SG_HX, 2 * SG_HY, 2.5)

    z_fl0 = hz_bot - flare_len
    if z_fl0 <= z_cv1:
        raise ValueError(f"segment {length} too short: tube would be "
                         f"{z_fl0 - z_cv1:.1f} mm")
    m += J.arm_tube(z_cv1 - OVL, z_fl0 + OVL)
    m += J.flare(z_fl0, hz_bot + OVL, J.arm_profile(), outer)
    m += house
    return m, cap, axis0, axis1


# ------------------------------------------------------------ SG90 cup ----
SG_HX = P.SG_H / 2.0 + P.SG_FIT + 2.5
SG_HY = P.SG_W / 2.0 + P.SG_FIT + 2.5


def sg_housing(axis_z):
    """SG90 cup. SG_FIT is the batch-variance knob -- clones are inconsistent
    enough that this is its own parameter, per the brief."""
    z_bot = axis_z - (P.SG_L - P.SG_SHAFT_OFF) - 2.5
    z_top = axis_z + P.SG_SHAFT_OFF + 6.0
    outer = pl.rounded_rect(2 * SG_HX, 2 * SG_HY, 2.5)
    bhx, bhy = P.SG_H / 2.0 + P.SG_FIT, P.SG_W / 2.0 + P.SG_FIT
    pocket = box(-bhx, -bhy, bhx, bhy)
    tab_x = P.SG_H / 2.0 - P.SG_TAB_Z
    tab_slot = box(tab_x - P.SG_TAB_T / 2 - 1.0, -bhy - 3.0,
                   tab_x + P.SG_TAB_T / 2 + 1.0, bhy + 3.0)
    spline = box(bhx - OVL, -4.5, SG_HX + 3.0, 4.5)
    m = pl.banded(outer, z_bot, z_top, [
        (pocket, z_bot + 2.5, z_top),
        (tab_slot, z_bot + 2.5 - P.SG_TAB_T - 0.4, z_bot + 2.5 + OVL),
        (spline, axis_z - 4.5, axis_z + 4.5),
    ])
    # Same threaded stub as the MG996R joints -- the shade is light but it
    # hangs off this sideways, and it had nothing holding it on either.
    _ax = pl.threaded_bore(P.AXLE_D, P.SCREW_MAJOR, P.SCREW_PITCH,
                           0.0, P.SCREW_ENGAGE, clearance=P.SCREW_FIT / 2)
    _ax.rotate_y(90.0)
    _ax.translate(dx=-SG_HX - P.AXLE_LEN + OVL, dz=axis_z)
    m += _ax
    return m, z_bot, z_top


def sg_cap(z_top):
    outer = pl.rounded_rect(2 * SG_HX, 2 * SG_HY, 2.5)
    holes = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            xa, xb = sorted((sx * (SG_HX - 4.0), sx * (SG_HX - 1.6)))
            ya, yb = sorted((sy * (SG_HY - 3.6), sy * (SG_HY - 1.6)))
            holes.append(box(xa, ya, xb, yb))
    return pl.prism(outer.difference(unary_union(holes)), z_top, z_top + 2.5)


def build():
    out = []
    lo, lo_cap, _, _ = segment(P.ARM_LOWER_L, "mg")
    up, up_cap, _, _ = segment(P.ARM_UPPER_L, "mg")
    fo, fo_cap, _, _ = segment(P.ARM_FORE_L, "sg")
    out += [("arm-lower", lo, P.COLORS["arm"]),
            ("arm-upper", up, P.COLORS["arm"]),
            ("arm-fore", fo, P.COLORS["arm"]),
            ("cap-shoulder", lo_cap, P.COLORS["joint"]),
            ("cap-elbow", up_cap, P.COLORS["joint"]),
            ("cap-head", fo_cap, P.COLORS["joint"])]
    return out


if __name__ == "__main__":
    ok = True
    for name, mesh, _c in build():
        r = pl.validate(mesh)
        fit, d = pl.fits_build_plate(mesh)
        print(f"{name:14s} shells={r['shells']:3d} tris={r['triangles']:6d} "
              f"watertight={r['watertight']} fits={fit} "
              f"bbox={tuple(round(float(v),1) for v in d)} {r['problems'][:2]}")
        ok &= r["watertight"] and fit
    sys.exit(0 if ok else 1)

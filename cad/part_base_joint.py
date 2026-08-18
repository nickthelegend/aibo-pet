"""part_base_joint.py — the base MG996R housing, and the arm's whole load path.

It bolts ON TOP of the lid (like the reference lamp, whose arm pivots above
the base surface) but its four M3 screws pass THROUGH the lid into heat-set
inserts in the two internal bulkheads -- so the arm's weight goes
housing -> bulkhead -> floor, and the lid skin carries nothing.

Modelled in the segment frame (servo L along Z, cup opening up = exactly how
it prints), then rotated -90 about X for the world: local +Z becomes world
+Y, so the cup opens toward the BACK and the servo slides in from behind
with the lamp assembled.
"""
from __future__ import annotations

import os
import sys

from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import joints as J
import params as P
import partlib as pl

OVL = pl.OVL
AXIS = 40.0                      # local Z of the pivot
WING_X = 40.0                    # flange wings reach
SCREW_Z = (22.0, 52.0)           # local Z -> world Y 0 and 30
PL = P.BJOINT_PLINTH             # standoff height under the housing
YK_CLEAR = J.YK_X0 - 1.0         # plinth must stop short of the yoke corridor


def build():
    cable = box(-11.0, J.H_HY - 3.0 - 1.0, 11.0, J.H_HY + 1.0)
    house, z_bot, z_top = J.housing(AXIS, extra=[(cable, 28.0, 42.0)])

    # Plinth: a centre block under the housing plus two outrigger wings, with
    # an open corridor between them at |X| 25.45..29.45 -- that is where the
    # yoke plates sweep. Without the gap the arm cannot rotate.
    y0, y1 = J.H_HY - OVL, J.H_HY + PL
    plinth = unary_union([
        box(-J.H_HX, y0, J.H_HX, y1),
        box(-WING_X, y0, -(J.YK_X1 + 1.0), y1),
        box(J.YK_X1 + 1.0, y0, WING_X, y1)])
    holes = []
    for cx in (-P.JOINT_BOLT_X, P.JOINT_BOLT_X):
        for cz in SCREW_Z:
            holes += J.hole_y(P.M3_CLEAR, y0 - OVL, y1 + OVL, cx, cz)
    holes.append((box(-11.0, y0 - OVL, 11.0, y1 + OVL), 28.0, 42.0))   # cable
    m = house
    m += pl.banded(plinth, 12.0, 58.0, holes)
    return [("base-joint", m, P.COLORS["joint"]),
            ("cap-base", J.housing_cap(AXIS, z_top), P.COLORS["joint"])]


def to_world(mesh):
    """Segment frame -> world: cup opens back, pivot at (0, BSERVO_AXIS_Y,
    BJOINT_AXIS_Z), flange face flat on the lid top."""
    return (mesh.copy().rotate_x(-90.0)
            .translate(dy=P.BSERVO_AXIS_Y - AXIS, dz=P.BJOINT_AXIS_Z))


if __name__ == "__main__":
    ok = True
    for n, m, _c in build():
        r = pl.validate(m)
        fit, d = pl.fits_build_plate(m)
        print(f"{n:12s} shells={r['shells']:3d} watertight={r['watertight']} "
              f"fits={fit} bbox={tuple(round(float(v),1) for v in d)} {r['problems'][:2]}")
        ok &= r["watertight"] and fit
    w = to_world(build()[0][1])
    b = w.bounds()
    print(f"  world bbox X {b[0]:.1f}..{b[3]:.1f}  Y {b[1]:.1f}..{b[4]:.1f}  "
          f"Z {b[2]:.1f}..{b[5]:.1f}   (lid top = {P.LID_Z1})")
    sys.exit(0 if ok else 1)

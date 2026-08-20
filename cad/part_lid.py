"""part_lid.py — round lid: the base's structural member.

With BULKHEADS on, this is just a lid: flat plate, MX key socket, four
clearance holes for the base joint's screws (which thread into the bulkhead
tops, not into anything here), a cable pass and a locating skirt. Nothing
hangs into the bay.

Set BULKHEADS = False and it grows a rib web on its underside instead, and
carries the arm itself -- see cad/audit_loads.py for which path is live.

Prints flat, top face DOWN, so the MX plate section and its cutout come out
crisp with nothing to support.

The MX detail is the point of the part: the switch clips into a plate that is
EXACTLY 1.5 thick, so the top 1.5 of the lid IS the plate (14.1 cutout) and
the 1.9 below opens out to 16.0 -- the relief the latches spring into. Push
the switch in from above; it pops in and stays.
"""
from __future__ import annotations

import os
import sys

from shapely import affinity
from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import partlib as pl

OVL = pl.OVL
# LID_OD, not BASE_TOP_D. At BASE_TOP_D the plate was the same diameter as
# the opening it covered, so it could only sit ON the shoulder -- a O118 disc
# standing 3.4 proud of a O118 rim. At LID_OD it drops INSIDE the bore, lands
# on the four seat lugs at Z58, and its top face finishes flush at Z61.4.
OUTER = pl.circle(P.LID_OD, 128)
Z0, Z1 = P.LID_Z0, P.LID_Z1
MX_PLATE_Z0 = Z1 - P.MX_PLATE_T


def build():
    mx = P.MX_CTR
    bolts = [(sx * P.JOINT_BOLT_X, y) for sx in (-1, 1) for y in P.JOINT_BOLT_Y]
    # With bulkheads fitted the inserts live in THEIR tops, so the lid just
    # passes the screws through. Without them the lid takes the inserts.
    jd = P.M3_CLEAR if P.BULKHEADS else P.M3_INSERT_D
    joint = unary_union([affinity.translate(pl.circle(jd), x, y) for x, y in bolts])
    # No rim screws any more. The lid is held by the snap bead and located by
    # three notches over the shoulder's keys; the lugs those screws went into
    # were the part that printed in mid air.
    cable = box(-11.0, P.BSERVO_AXIS_Y - 7.0, 11.0, P.BSERVO_AXIS_Y + 7.0)
    mx_cut = affinity.translate(pl.rounded_rect(P.MX_CUT, P.MX_CUT, P.MX_CORNER_R), *mx)
    mx_rel = affinity.translate(pl.rounded_rect(P.MX_BODY_SQ, P.MX_BODY_SQ, 1.0), *mx)

    import part_shoulder as _sh
    notches = _sh._key_profile((P.BASE_TOP_D - 2 * P.WALL_STRUCT) / 2.0,
                               P.LID_KEY_FIT)
    m = pl.banded(OUTER.difference(notches), Z0, Z1, [
        (unary_union([joint, cable]), Z0, Z1),
        (mx_rel, Z0, MX_PLATE_Z0),
        (mx_cut, MX_PLATE_Z0, Z1),
    ])
    # The rib web only exists when there are NO bulkheads. With them fitted
    # the base joint bolts straight into their tops and the arm's load never
    # touches the lid, so ribs here would be dead plastic hanging into the
    # bay for nothing.
    if not P.BULKHEADS:
        m += _ribs(bolts, unary_union([joint, cable, mx_rel]))
    # Skirt sized to the base's STRAIGHT rebate (constant bore), not to a
    # cone -- a skirt dropping into a taper only fits at one height, and the
    # first attempt fouled the bore by 2.1 mm at the top. It is no longer what
    # locates the lid -- the recessed plate does that now -- but it still
    # carries the snap bead down to the groove at Z55, below the seat.
    seat_in = P.BASE_TOP_D - 2 * P.WALL_STRUCT
    sk_out = pl.circle(min(seat_in - 0.3, P.LID_OD - 0.2), 128)
    skirt = pl.ring2d(sk_out, pl.circle(seat_in - 0.3 - 2 * 1.4, 128))
    bead = pl.ring2d(pl.circle(seat_in - 0.3 + 2 * P.SNAP_BEAD, 128),
                     pl.circle(seat_in - 0.3 - 2 * 1.4, 128))
    hole = unary_union([mx_rel, cable])
    m += pl.banded(skirt.difference(hole), P.LID_SEAT_Z, Z0 + OVL, [])
    m += pl.prism(bead.difference(hole), P.SNAP_Z + 0.3, P.SNAP_Z + 1.3)
    return [("lid", m, P.COLORS["lid"])]


def _ribs(bolts, holes):
    """Underside stiffening web: a frame through the four bolt bosses plus
    radial ribs out to the rim. This is what replaced the bulkheads."""
    t = P.LID_RIB_T
    seg = [((-P.JOINT_BOLT_X, P.JOINT_BOLT_Y[0]), (P.JOINT_BOLT_X, P.JOINT_BOLT_Y[0])),
           ((-P.JOINT_BOLT_X, P.JOINT_BOLT_Y[1]), (P.JOINT_BOLT_X, P.JOINT_BOLT_Y[1])),
           ((-P.JOINT_BOLT_X, P.JOINT_BOLT_Y[0]), (-P.JOINT_BOLT_X, P.JOINT_BOLT_Y[1])),
           ((P.JOINT_BOLT_X, P.JOINT_BOLT_Y[0]), (P.JOINT_BOLT_X, P.JOINT_BOLT_Y[1])),
           ((-P.JOINT_BOLT_X, P.JOINT_BOLT_Y[0]), (-50.0, -20.0)),
           ((P.JOINT_BOLT_X, P.JOINT_BOLT_Y[0]), (50.0, -20.0)),
           ((-P.JOINT_BOLT_X, P.JOINT_BOLT_Y[1]), (-46.0, 46.0)),
           ((P.JOINT_BOLT_X, P.JOINT_BOLT_Y[1]), (46.0, 46.0))]
    web = unary_union([pl.stroke([a, b], t) for a, b in seg]
                      + [affinity.translate(pl.circle(P.M3_BOSS_D + 2.0), x, y)
                         for x, y in bolts])
    web = web.intersection(pl.circle(106.0, 128)).difference(holes)
    return pl.prism(web, P.LID_RIB_Z0, Z0 + OVL)


if __name__ == "__main__":
    ok = True
    for n, m, _c in build():
        r = pl.validate(m)
        fit, d = pl.fits_build_plate(m)
        print(f"{n}: shells={r['shells']} watertight={r['watertight']} fits={fit} "
              f"bbox={tuple(round(float(v),1) for v in d)} {r['problems'][:3]}")
        ok &= r["watertight"] and fit
    sys.exit(0 if ok else 1)

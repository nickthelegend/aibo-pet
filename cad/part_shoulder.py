"""part_shoulder.py — the tapered ring between tub and lid.

This is a separate part for one reason: assembly. Moulded onto the tub it
closes the mouth to O113 while the interior is O155, and the speaker -- which
sits against the wall at radius 74.6 -- physically cannot be got into its
pocket past it. Nothing near the wall can. Split here and the tub is open to
its full bore, so every component drops straight down into place, then this
ring goes on.

  42 .. 45   inward flange, 3 M3 clearance holes down into the tub's inserts
  36 .. 42   locating skirt dropping inside the tub bore
  42 .. 52   quarter-sine taper, O160 -> O118, tangent to the tub at the rim
  52 .. 58   straight lid rebate + the lid's snap groove
  ..         4 seat lugs the lid screws into

Prints as modelled, sitting on its wide rim: the taper only ever narrows
going up, so the whole thing is self-supporting.
"""
from __future__ import annotations

import math
import os
import sys

from shapely import affinity
from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import partlib as pl

OVL = pl.OVL
SEAT_IN = P.BASE_TOP_D - 2 * P.WALL_STRUCT      # 113.2, constant lid bore
FLANGE_R = 68.0
SKIRT_Z0 = 36.0


def shoulder_od(z):
    """Quarter-sine: tangent to the tub's cylinder where it leaves it, so the
    two read as one turned form instead of meeting at a hard step."""
    t = min(max((z - P.BASE_STRAIGHT) / (P.LID_SEAT_Z - P.BASE_STRAIGHT), 0.0), 1.0)
    return P.BASE_D - (P.BASE_D - P.BASE_TOP_D) * math.sin(t * math.pi / 2)


def build():
    m = pl.revolve_shell(P.BASE_STRAIGHT, P.LID_SEAT_Z, shoulder_od,
                         P.WALL_STRUCT, steps=24)
    ring = pl.ring2d(pl.circle(P.BASE_TOP_D, 128), pl.circle(SEAT_IN, 128))
    groove = pl.ring2d(pl.circle(P.BASE_TOP_D, 128),
                       pl.circle(SEAT_IN + 2 * P.SNAP_BEAD, 128))
    m += pl.banded(ring, P.LID_SEAT_Z - OVL, P.BASE_H,
                   [(groove, P.SNAP_Z, P.SNAP_Z + 1.6)])

    # inward flange with the tub screws
    inner_at_rim = P.BASE_D / 2 - P.WALL_STRUCT
    flange = pl.ring2d(pl.circle(2 * (inner_at_rim + OVL), 128),
                       pl.circle(2 * FLANGE_R, 128))
    holes = unary_union([affinity.translate(pl.circle(P.M3_CLEAR), x, y)
                         for x, y in P.SHOULDER_POS])
    heads = unary_union([affinity.translate(pl.circle(P.M3_HEAD_D), x, y)
                         for x, y in P.SHOULDER_POS])
    m += pl.banded(flange, P.BASE_STRAIGHT - OVL, P.BASE_STRAIGHT + 3.0,
                   [(holes, P.BASE_STRAIGHT, P.BASE_STRAIGHT + 3.0),
                    (heads, P.BASE_STRAIGHT + 1.2, P.BASE_STRAIGHT + 3.0)])

    # locating skirt into the tub bore
    sk = pl.ring2d(pl.circle(2 * (inner_at_rim - 0.3), 128),
                   pl.circle(2 * (inner_at_rim - 1.7), 128))
    m += pl.prism(sk, SKIRT_Z0, P.BASE_STRAIGHT + OVL)

    # lid hold-down lugs
    for cx, cy in P.LUG_POS:
        a = math.atan2(cy, cx)
        prof = affinity.translate(pl.circle(P.M3_BOSS_D), cx, cy).union(
            pl.stroke([(cx, cy), (cx + 12 * math.cos(a), cy + 12 * math.sin(a))], 3.4))
        prof = prof.intersection(pl.circle(SEAT_IN + 2 * OVL, 128))
        bore = affinity.translate(pl.circle(P.M3_INSERT_D), cx, cy)
        m += pl.prism(prof, P.LUG_Z0, P.LID_SEAT_Z)
        m += pl.prism(prof.difference(bore), P.LID_SEAT_Z - OVL, P.BASE_H)
    return [("shoulder", m, P.COLORS["base"])]


if __name__ == "__main__":
    ok = True
    for n, mm, _c in build():
        r = pl.validate(mm)
        fit, d = pl.fits_build_plate(mm)
        print(f"{n}: shells={r['shells']} tris={r['triangles']} "
              f"watertight={r['watertight']} fits={fit} "
              f"bbox={tuple(round(float(v),1) for v in d)} {r['problems'][:3]}")
        ok &= r["watertight"] and fit
    sys.exit(0 if ok else 1)

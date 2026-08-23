"""part_shoulder.py — the tapered ring between tub and lid.

This is a separate part for one reason: assembly. Moulded onto the tub it
closes the mouth to O113 while the interior is O155, and the speaker -- which
sits against the wall at radius 74.6 -- physically cannot be got into its
pocket past it. Nothing near the wall can. Split here and the tub is open to
its full bore, so every component drops straight down into place, then this
ring goes on.

  34 .. 37   inward flange, 3 M3 clearance holes down into the tub's inserts
  28 .. 34   locating skirt dropping inside the tub bore
  34 .. 52   STRAIGHT taper, O160 -> O118, 47.9 deg off vertical. It was a
             quarter-sine over 42..52 -- prettier, unprintable at 73.1 deg.
  52 .. 61.4 straight lid rebate + the lid's snap groove. It carries on past
             the 58 seat by the lid's 3.4 thickness, so the lid drops INSIDE
             the ring and the two top faces finish flush -- no proud step.
  ..         4 seat lugs at r48, tops at 58: what the lid lands on and screws
             into. At r53 their counterbores broke out of the recessed lid.

PRINTS UPSIDE DOWN, on the lid rebate rim. It does not print as modelled:
the four lid lugs cantilever into the bore with flat undersides 20 mm above
the bed, and the flange overhangs the skirt it would stand on. That is
10,270 mm2 over thin air, and it is how the first one came out with its
screw holes hanging in space. Inverted it is 287 mm2. The taper then runs
as a 45 degree overhang, which costs surface finish and nothing else.
assembly.PRINT_FLIP carries the rotation; audit_support.py measures it.
"""
from __future__ import annotations

import math
import os
import sys

from shapely import affinity
import numpy as np
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import partlib as pl

OVL = pl.OVL
SEAT_IN = P.BASE_TOP_D - 2 * P.WALL_STRUCT      # 113.2, constant lid bore
FLANGE_R = 68.0
SKIRT_Z0 = P.BASE_STRAIGHT - 6.0   # 6 mm of skirt DOWN inside the tub bore.

# The tub's mic boss does not stop at the rim. It runs up to Z41, seven
# millimetres proud, over 262..278 degrees at r74.2..78.5, and the ring lands
# straight on it -- which is why the first shoulder sat tilted and would not
# seat however hard it was pressed. Measured with cad/solidtest.py, not
# guessed. The tub is already printed and is not changing, so the ring gets
# the relief, and it doubles as the acoustic window the mic needed anyway.
MIC_A0, MIC_A1 = 256.0, 284.0      # sector, with 6 deg of margin each side
MIC_R = 72.4                       # cut inward to here, clear of r74.2
MIC_Z1 = 42.0                      # clear of the boss top at Z41

# Three locating pads, replacing the continuous skirt that could not fit.
# The tub bore is free out to r77.75 over most of the circle, but bulkheads
# and boss ribs reach in past r75.9 in six narrow sectors, which is what the
# full ring fouled on. These sit only where the bore was MEASURED clear, at
# roughly 120 degree spacing, and away from the mic relief.
# Without them nothing centres the ring: the three M3s pass through 3.2 mm
# clearance holes, which stop it sliding once tightened but do not locate it.
# Deleting the skirt outright and calling the screws a locator was wrong.
PAD_ANGLES = (60.0, 185.0, 299.0)
PAD_ARC = 22.0                     # degrees each
PAD_OD_R = 77.5                    # 0.25 clear of the free bore at r77.75
PAD_T = 1.4
                                   # Was hardcoded 36, which sat above the rim
                                   # once the taper moved down to 34 and turned
                                   # the skirt inside out.


def shoulder_od(z):
    """Straight cone.

    It was a quarter-sine, tangent to the tub at the rim so the two read as
    one turned form. It looked better and it could not be printed: a sine
    puts its STEEPEST slope right at the rim, pi/2 times the average, which
    over a 10 mm taper is 73.1 degrees off vertical. On a shell that is 5,700
    mm2 of unsupported inner surface, and flipping the part rim-down only
    moves it to the outer surface and makes it worse (6,660 mm2, measured).

    Linear spends the same height at a constant angle instead of front-
    loading it, and starting at Z34 rather than Z42 buys 8 mm more run.
    73.1 -> 47.9 degrees. Under 45 would need either a wider neck or a taller
    base -- the speaker pocket tops out at Z32.4 and the wall has to stay
    straight past it, so 34 is the floor."""
    t = min(max((z - P.BASE_STRAIGHT) / (P.LID_SEAT_Z - P.BASE_STRAIGHT), 0.0), 1.0)
    return P.BASE_D - (P.BASE_D - P.BASE_TOP_D) * t


def _mic_relief():
    """The sector cut out of the ring so the tub's mic boss can pass."""
    pts = [(0.0, 0.0)]
    for a in np.linspace(math.radians(MIC_A0), math.radians(MIC_A1), 40):
        pts.append((95.0 * math.cos(a), 95.0 * math.sin(a)))
    return Polygon(pts).difference(pl.circle(2 * MIC_R, 128))


def _key_profile(seat_r, fit):
    """The three keys, grown by `fit` per side. ONE definition, used by the
    shoulder to add them and by the lid to cut its notches, so the two halves
    cannot drift apart the way two copies of the same numbers always do."""
    out = []
    for k in range(P.LID_KEY_N):
        a = 90.0 + k * (360.0 / P.LID_KEY_N)
        w = P.LID_KEY_W / 2.0 + fit
        tab = box(-w, seat_r - P.LID_KEY_D - fit, w, seat_r + 0.8)
        out.append(affinity.rotate(tab, a - 90.0, origin=(0, 0)))
    return unary_union(out)


def build():
    relief = _mic_relief()
    # Taper in two pieces. The stretch that has to clear the boss is a stack
    # of banded rings so the sector can be differenced out of each one;
    # revolve_shell makes a single lofted shell that cannot be cut, this
    # kernel having no 3D CSG. Above the boss it goes back to the smooth loft.
    steps = 16
    m = pl.Mesh()
    for k in range(steps):
        za = P.BASE_STRAIGHT + k * (MIC_Z1 - P.BASE_STRAIGHT) / steps
        zb = P.BASE_STRAIGHT + (k + 1) * (MIC_Z1 - P.BASE_STRAIGHT) / steps + OVL
        od = shoulder_od(za)
        ring = pl.ring2d(pl.circle(od, 128), pl.circle(od - 2 * P.WALL_STRUCT, 128))
        m += pl.prism(ring.difference(relief), za, zb)
    m += pl.revolve_shell(MIC_Z1 - OVL, P.LID_SEAT_Z, shoulder_od,
                          P.WALL_STRUCT, steps=16)
    ring = pl.ring2d(pl.circle(P.BASE_TOP_D, 128), pl.circle(SEAT_IN, 128))
    # The snap groove recesses the BORE and leaves the outside alone. It was
    # the other way round -- ring2d(BASE_TOP_D, SEAT_IN + 2*SNAP_BEAD), the
    # OUTER slice of the wall -- which cut a 1.95 mm channel right round the
    # outside of the base and left a 0.45 mm wall behind it, under one 0.4
    # perimeter. Cutting the inner slice leaves 1.95 mm of wall and puts the
    # groove where the lid's bead actually runs.
    groove = pl.ring2d(pl.circle(SEAT_IN + 2 * P.SNAP_BEAD, 128),
                       pl.circle(SEAT_IN, 128))
    # Rebate wall runs up to LID_Z1, not BASE_H -- it has to rise PAST the
    # seat by the lid's own thickness so the lid sits down inside it and the
    # two finish level. Stopping at BASE_H is what left the lid perched proud.
    m += pl.banded(ring, P.LID_SEAT_Z - OVL, P.LID_Z1,
                   [(groove, P.SNAP_Z, P.SNAP_Z + 1.6)])

    # inward flange with the tub screws
    inner_at_rim = P.BASE_D / 2 - P.WALL_STRUCT
    flange = pl.ring2d(pl.circle(2 * (inner_at_rim + OVL), 128),
                       pl.circle(2 * FLANGE_R, 128))
    holes = unary_union([affinity.translate(pl.circle(P.M3_CLEAR), x, y)
                         for x, y in P.SHOULDER_POS])
    heads = unary_union([affinity.translate(pl.circle(P.M3_HEAD_D), x, y)
                         for x, y in P.SHOULDER_POS])
    # Starts AT the rim, not OVL below it. That 0.2 mm is a trick for fusing
    # shells WITHIN one part; between two separate parts it is interference,
    # and it was the last thing holding the ring off the tub. It still fuses
    # upward into the taper, which also starts at BASE_STRAIGHT.
    m += pl.banded(flange.difference(relief), P.BASE_STRAIGHT, P.BASE_STRAIGHT + 3.0,
                   [(holes, P.BASE_STRAIGHT, P.BASE_STRAIGHT + 3.0),
                    (heads, P.BASE_STRAIGHT + 1.2, P.BASE_STRAIGHT + 3.0)])

    # ---- three locating pads ----
    pads = []
    ring = pl.ring2d(pl.circle(2 * PAD_OD_R, 256),
                     pl.circle(2 * (PAD_OD_R - PAD_T), 256))
    for c in PAD_ANGLES:
        pts = [(0.0, 0.0)]
        for a in np.linspace(math.radians(c - PAD_ARC / 2),
                             math.radians(c + PAD_ARC / 2), 24):
            pts.append((95.0 * math.cos(a), 95.0 * math.sin(a)))
        pads.append(ring.intersection(Polygon(pts)))
    m += pl.prism(unary_union(pads), SKIRT_Z0, P.BASE_STRAIGHT + OVL)

    # ---- the continuous skirt is gone ----
    # It used to drop a 6 mm ring at r75.9..77.2 into the tub bore. The tub's
    # internals do not leave that annulus free: bulkheads and boss ribs reach
    # inward past r75.9 over about a fifth of the circumference, as far as r66
    # near 166 degrees. A continuous skirt clearing all of it would need an OD
    # of r65.6, which locates nothing in a O155 bore. It was fouling on 15 of
    # 68 probed angles and is a large part of why the ring sat proud.
    # The three M3s at r72 locate the ring; the rim carries it.

    # ---- lid seat: a continuous ledge, not four lugs ----
    # The lugs are gone. They reached 12 mm into the bore from a wall that
    # recedes at exactly 45 degrees, so nothing could support them from below
    # and they printed as string. A ring seat has none of that problem: it
    # steps inward SEAT_LEDGE_W from the rebate bore and is relieved beneath
    # at 45 degrees, so it self supports, and it carries the lid all the way
    # round instead of at four points.
    seat_r = SEAT_IN / 2.0
    inner_r = seat_r - P.SEAT_LEDGE_W
    # Built as a short stack of rings rather than one loft, for two reasons.
    # loft_solid takes hole-free profiles, so lofting circle to circle here
    # produced a SOLID frustum that plugged the whole bore with a O109 disc.
    # And the ledge has to reach the wall at every height: the taper bore is
    # 111.6 - z, so a ring of constant outer radius floats free 1 mm inside it
    # a millimetre below the seat, which is the same defect as the lugs.
    # Each ring therefore runs from the LOCAL bore inward to a constant
    # inner_r, and consecutive rings overlap so the slicer fuses them.
    steps = 8
    for k in range(steps):
        za = P.LID_SEAT_Z - (k + 1) * (P.SEAT_RAMP + 1.2) / steps
        # Top ring stops AT the seat plane. The usual +OVL is for fusing
        # shells within a part; here it put 0.2 mm of ledge into the lid's
        # space and held the lid proud. Lower rings still overlap upward.
        zb = min(P.LID_SEAT_Z,
                 P.LID_SEAT_Z - k * (P.SEAT_RAMP + 1.2) / steps + OVL)
        bore_r = shoulder_od(za) / 2.0 - P.WALL_STRUCT
        m += pl.prism(pl.ring2d(pl.circle(2 * (bore_r + OVL), 128),
                                pl.circle(2 * inner_r, 128)), za, zb)

    # ---- three keys, so it cannot turn ----
    # A slip fit never stops rotation however tight it is; the four screws
    # used to. These protrude INWARD from the rebate bore and drop into
    # notches in the lid's rim. Keys on the ring and notches in the lid, not
    # the other way round, because this kernel does no 3D CSG: adding a prism
    # is free, cutting a slot into an already built ring is not.
    # Vertical walls top to bottom, so they add no overhang at all.
    m += pl.prism(_key_profile(seat_r, 0.0), P.LID_Z0 - OVL, P.LID_Z1)

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

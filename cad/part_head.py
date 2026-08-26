"""part_head.py — the cone. A proper Pixar shade, not a bowl.

Wide mouth, long taper, apex behind, tilt pivot BEHIND the cone -- which is
the only place a yoke can straddle a cone without burying itself in it. The
taper narrows as Z rises, so the whole shell is self-supporting; the apex is
left OPEN, which kills the last bridge and doubles as the ring's wire exit.

Stack, mouth (Z=0) up:
   0.0 ..  3.0   bezel      lip retaining the WS2812 ring; glow exits here
   3.0 ..  5.6   seat       RING_OD + fit pocket
   5.6 .. 30.0   cone
  30.0 .. 36.0   vents      SHADE_VENTS slots, like the reference lamp
  36.0 .. 52.0   cone
  42.0 .. 52.0   collar     flares outward at 38 deg to carry the yoke
  52.0 .. TILT+12 yoke      drive plate (+X, MG996R horn cross) / idler (-X)

The tilt joint is an MG996R, not the SG90 this cone was first cut for. Two
things follow, and the second was a defect the whole time: the yoke has to
straddle a 52.3 wide nose instead of a 36.6 one, and it has to reach PAST
the pivot. It did not. Z_YOKE1 was Z_CONE1 + 20 = 72 with the axis at 74,
so the horn cross recess was clipped to a 0.25 sliver and the cone was
hanging on nothing. Every static audit passed it, because a cone attached
to nothing collides with nothing.

Printed mouth DOWN, exactly as modelled.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
from shapely import affinity
from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import joints as J
import params as P
import partlib as pl

OVL = pl.OVL
Z_SEAT0, Z_SEAT1 = 3.0, 3.0 + P.RING_T + 0.4
Z_VENT0, Z_VENT1 = 30.0, 36.0
Z_CONE1 = P.SHADE_DEPTH
Z_COLLAR0 = Z_CONE1 - P.SHADE_COLLAR
TILT = P.SHADE_TILT_Z
# past the axis by the hub radius plus a wall, so the bore is IN the plate
Z_YOKE1 = TILT + P.HORN_HUB_D / 2 + 5.5
RING_AP = P.RING_ID + 10.0
# the yoke straddles the head nose at its DRIVE CHEEK face -- case centre to
# case top plus the boss, the same plane link1's plate sits 0.3 outside of
YK0 = (P.MG_H / 2 + P.MG_BOSS_H) + 0.3
YK1 = YK0 + P.YOKE_PLATE_T
YK_HY = P.HORN_HUB_D / 2 + 9.5      # material round the O13 hub


# The vent band is a STRAIGHT collar, not part of the taper. A tapered band
# has to be built as stacked rings (the kernel extrudes straight walls only),
# and stacked rings of changing diameter show up as ridges through the slots.
# Making it a true cylinder removes the stepping entirely and reads as a
# deliberate vent collar. The two tapered sections either side are given the
# same angle so the silhouette stays one continuous cone.
_FRAC = (Z_VENT0 - Z_SEAT1) / ((Z_VENT0 - Z_SEAT1) + (Z_CONE1 - Z_VENT1))
OD_VENT = P.SHADE_OD - (P.SHADE_OD - P.SHADE_APEX_D) * _FRAC


def od(z):
    if z <= Z_VENT0:
        t = min(max((z - Z_SEAT1) / (Z_VENT0 - Z_SEAT1), 0.0), 1.0)
        return P.SHADE_OD * (1 - t) + OD_VENT * t
    if z <= Z_VENT1:
        return OD_VENT
    t = min(max((z - Z_VENT1) / (Z_CONE1 - Z_VENT1), 0.0), 1.0)
    return OD_VENT * (1 - t) + P.SHADE_APEX_D * t


def cone_shell(z0, z1, wall, seg=96):
    return pl.revolve_shell(z0, z1, od, wall, steps=2, seg=seg)


def _vents():
    """Slots round the taper -- the reference lamp has them, and the ring
    needs somewhere to dump its heat."""
    cuts = []
    for a in np.linspace(0, 2 * math.pi, P.SHADE_VENTS, endpoint=False):
        cuts.append(affinity.rotate(box(-2.0, 10.0, 2.0, 40.0), math.degrees(a),
                                    origin=(0, 0), use_radians=False))
    ring = pl.ring2d(pl.circle(OD_VENT, 96), pl.circle(OD_VENT - 2 * P.SHADE_WALL, 96))
    return pl.prism(ring.difference(unary_union(cuts)), Z_VENT0 - OVL, Z_VENT1 + OVL)


def _collar():
    """Outward flare from the cone to the yoke plates, 38 deg -- printable."""
    def collar_od(z):
        t = (z - Z_COLLAR0) / (Z_CONE1 - Z_COLLAR0)
        return od(z) * (1 - t) + (2 * YK1) * t

    def collar_wall(z):
        return (collar_od(z) - (od(z) - 2 * P.SHADE_WALL)) / 2.0

    zs = np.linspace(Z_COLLAR0, Z_CONE1, 25)
    m = pl.Mesh(weld=True)
    outer = [pl.circle(collar_od(z), 96) for z in zs]
    inner = [pl.circle(od(z) - 2 * P.SHADE_WALL, 96) for z in zs]
    for i in range(len(zs) - 1):
        m.add_loft_wall(pl._rings(outer[i])[0], zs[i],
                        pl._rings(outer[i + 1])[0], zs[i + 1])
        m.add_loft_wall(pl._rings(inner[i])[0][::-1], zs[i],
                        pl._rings(inner[i + 1])[0][::-1], zs[i + 1])
    m.add_cap(pl.ring2d(outer[-1], inner[-1]), zs[-1], up=True)
    m.add_cap(pl.ring2d(outer[0], inner[0]), zs[0], up=False)
    return m


def _mouth():
    """Mouth lip plus a SPOKED carrier for the LED ring.

    The mouth used to be solid from the ring out to the rim. At a 66 mouth
    that was tolerable; at 88 it is a 43 mm wide flat disc of PLA -- about
    31 g, hung 74 mm off the tilt axis, to retain a 12 g ring. Three spokes
    do the same job, and the rest of the mouth is the aperture the light
    actually leaves through.

    Ring fitting: it drops in from the APEX side and lands on the aperture
    lip; the pocket wall round it takes the sideways load. Wires leave up
    the open apex.
    """
    wall = P.SHADE_WALL
    shelf_id = P.RING_OD + P.RING_FIT
    shelf_od = shelf_id + 2 * wall + 1.4
    m = pl.prism(pl.ring2d(pl.circle(P.SHADE_OD, 96),
                           pl.circle(P.SHADE_OD - 2 * wall, 96)), 0.0, Z_SEAT1)
    m += pl.prism(pl.ring2d(pl.circle(shelf_od, 96), pl.circle(RING_AP, 96)),
                  0.0, Z_SEAT0)                       # the lip the ring sits on
    m += pl.prism(pl.ring2d(pl.circle(shelf_od, 96), pl.circle(shelf_id, 96)),
                  Z_SEAT0 - OVL, Z_SEAT1)             # pocket wall round it
    spokes = [affinity.rotate(box(-4.0, shelf_od / 2 - 1.0, 4.0, P.SHADE_OD / 2),
                              a, origin=(0, 0))
              for a in (0.0, 120.0, 240.0)]
    m += pl.prism(unary_union(spokes), 0.0, Z_SEAT1)
    return m


def build():
    m = pl.Mesh()
    m += _mouth()
    m += cone_shell(Z_SEAT1 - OVL, Z_VENT0 + OVL, P.SHADE_WALL)
    m += _vents()
    m += cone_shell(Z_VENT1 - OVL, Z_CONE1, P.SHADE_WALL)
    m += _collar()

    # drive yoke: the MG996R cross interface as DROP-IN CHANNELS. A closed
    # cross recess plus a closed hub bore is geometry the shade can never
    # reach: sliding sideways drives the idler plate through the head, and
    # dropping vertically drives the horn's horizontal arm and proud hub
    # through solid plate (the closure audit walked both). So every opening
    # runs out through the yoke tip -- the shade drops straight down over
    # the mounted horn, arms in their channels, hub in its slot -- and the
    # retention that the open slots give up is put back explicitly: a glued
    # cap over the hub end (drive) and an M3 + washer into the stub's pilot
    # (idler). Torque rides the channel side walls exactly as before.
    hy = YK_HY
    aw = P.HORN_ARM_W + P.HORN_FIT
    hub = P.HORN_HUB_D + P.HORN_FIT
    rec_d = P.HORN_T + P.HORN_FIT
    rec = box(YK0 - OVL, -hy - 1, YK0 + rec_d, hy + 1)
    m += pl.banded(box(YK0, -hy, YK1, hy), Z_CONE1 - OVL, Z_YOKE1, [
        # horizontal arm: its channel runs from the axis out the tip
        (rec.intersection(box(-99, -P.HORN_ARM_HALF, 99, P.HORN_ARM_HALF)),
         TILT - aw / 2, Z_YOKE1 + OVL),
        # vertical arm: full height by its own span
        (rec.intersection(box(-99, -aw / 2, 99, aw / 2)),
         TILT - P.HORN_ARM_HALF, TILT + P.HORN_ARM_HALF),
        # hub: keyhole slot clear through, axis to tip
        (box(YK0 - OVL, -hub / 2, YK1 + OVL, hub / 2),
         TILT - hub / 2, Z_YOKE1 + OVL),
    ])
    slot_w = (P.AXLE_D + P.AXLE_FIT) / 2.0
    m += pl.banded(box(-YK1, -hy, -YK0, hy), Z_CONE1 - OVL, Z_YOKE1,
                   [(box(-YK1 - OVL, -w, -YK0 + OVL, w), zl, zh)
                    for zl, zh, w in J._disc_slices(P.AXLE_D + P.AXLE_FIT, 0.0, TILT)]
                   + [(box(-YK1 - OVL, -slot_w, -YK0 + OVL, slot_w),
                       TILT, Z_YOKE1 + OVL)])
    return [("shade", m, P.COLORS["shade"])]


if __name__ == "__main__":
    ok = True
    for n, m, _c in build():
        r = pl.validate(m)
        fit, d = pl.fits_build_plate(m)
        print(f"{n}: shells={r['shells']} tris={r['triangles']} "
              f"watertight={r['watertight']} fits={fit} "
              f"bbox={tuple(round(float(v),1) for v in d)} {r['problems'][:3]}")
        ok &= r["watertight"] and fit
    print(f"  cone {P.SHADE_OD} -> {P.SHADE_APEX_D} over {P.SHADE_DEPTH}, "
          f"cone half-angle {math.degrees(math.atan2((P.SHADE_OD-P.SHADE_APEX_D)/2, P.SHADE_DEPTH)):.1f} deg")
    sys.exit(0 if ok else 1)

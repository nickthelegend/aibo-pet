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
  52.0 .. 72.0   yoke       drive plate (+X, SG90 horn cross) / idler (-X)

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
import part_arms as PA
import partlib as pl

OVL = pl.OVL
Z_SEAT0, Z_SEAT1 = 3.0, 3.0 + P.RING_T + 0.4
Z_VENT0, Z_VENT1 = 30.0, 36.0
Z_CONE1 = P.SHADE_DEPTH
Z_COLLAR0 = Z_CONE1 - P.SHADE_COLLAR
Z_YOKE1 = Z_CONE1 + 20.0
TILT = P.SHADE_TILT_Z
RING_AP = P.RING_ID + 10.0
YK0 = PA.SG_HX + P.JOINT_GAP
YK1 = YK0 + P.YOKE_PLATE_T


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


def build():
    m = pl.Mesh()
    m += pl.prism(pl.ring2d(pl.circle(P.SHADE_OD, 96), pl.circle(RING_AP, 96)), 0.0, Z_SEAT0)
    m += pl.prism(pl.ring2d(pl.circle(od(Z_SEAT0), 96),
                            pl.circle(P.RING_OD + P.RING_FIT, 96)), Z_SEAT0 - OVL, Z_SEAT1)
    m += cone_shell(Z_SEAT1 - OVL, Z_VENT0 + OVL, P.SHADE_WALL)
    m += _vents()
    m += cone_shell(Z_VENT1 - OVL, Z_CONE1, P.SHADE_WALL)
    m += _collar()

    hy = 9.0
    rec = box(YK0 - OVL, -hy - 1, YK0 + P.SG_HORN_T + 0.3, hy + 1)
    aw, hub = P.SG_HORN_W + 0.3, P.SG_HORN_HUB_D + 0.4
    m += pl.banded(box(YK0, -hy, YK1, hy), Z_CONE1 - OVL, Z_YOKE1, [
        (rec, TILT - aw / 2, TILT + aw / 2),
        (rec.intersection(box(-99, -aw / 2, 99, aw / 2)),
         TILT - P.SG_HORN_ARM, TILT + P.SG_HORN_ARM),
        (box(YK0 - OVL, -hub / 2, YK1 + OVL, hub / 2), TILT - hub / 2, TILT + hub / 2),
    ])
    m += pl.banded(box(-YK1, -hy, -YK0, hy), Z_CONE1 - OVL, Z_YOKE1,
                   [(box(-YK1 - OVL, -w, -YK0 + OVL, w), zl, zh)
                    for zl, zh, w in J._disc_slices(P.M3_CLEAR + 1.0, 0.0, TILT)])
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

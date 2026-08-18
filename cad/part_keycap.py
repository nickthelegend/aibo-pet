"""part_keycap.py — the MX keycap.

The lid has carried an MX socket from the start (a 14.1 cutout through a
plate section held at exactly 1.5 so the switch's clips catch) and nothing
ever went into it. This is the cap that finishes it.

A 1u square tapering KEYCAP_BASE -> KEYCAP_TOP over KEYCAP_H, hollow, with an
MX cross socket up the middle.

Built the way the kernel wants (cad/README.md): no 3D CSG. The skirt is a
stack of WALL bands -- each one a 2D ring extruded through its own slice of
the taper -- capped by a solid roof, so it comes out as one closed shell
rather than a solid with a cavity subtracted from it.

Prints TOP FACE DOWN, and only that way:

  - the top face is the surface you actually look at, and on the bed it is
    flat and seamless
  - the walls flare OUTWARD going up, atan(((18 - 13.4)/2) / 9) = 14.3 deg
    off vertical, nowhere near the 45 that would need support
  - the cross stem grows UP off the inside of the top face, supported by the
    layer under it the whole way

Flipped, it would need support inside the shell and put the seam on the top.
"""
from __future__ import annotations

import os
import sys

from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import partlib as pl

OVL = pl.OVL
STEPS = 16


def _cross(a, b):
    """MX stem cross: two crossed slots, centred."""
    return unary_union([box(-a / 2, -b / 2, a / 2, b / 2),
                        box(-b / 2, -a / 2, b / 2, a / 2)])


def _side(t):
    """Cap width at height fraction t through the taper."""
    return P.KEYCAP_BASE + t * (P.KEYCAP_TOP - P.KEYCAP_BASE)


def build():
    z1 = P.KEYCAP_H
    roof_z = z1 - P.KEYCAP_WALL          # underside of the solid top
    m = pl.Mesh()

    for i in range(STEPS):
        t0, t1 = i / STEPS, (i + 1) / STEPS
        za, zb = t0 * z1, t1 * z1
        if za >= roof_z:                 # solid roof: loft the full section
            m += pl.loft_solid(pl.rounded_rect(_side(t0), _side(t0), 1.2, seg=8), za,
                               pl.rounded_rect(_side(t1), _side(t1), 1.2, seg=8), zb)
            continue
        zb = min(zb, roof_z)             # skirt: a wall ring through this band
        s = _side((t0 + t1) / 2)
        ring = pl.ring2d(pl.rounded_rect(s, s, 1.2, seg=8),
                         pl.rounded_rect(s - 2 * P.KEYCAP_WALL,
                                         s - 2 * P.KEYCAP_WALL, 0.8, seg=8))
        m += pl.prism(ring, za, zb)

    # Stem boss hanging off the roof, cross bored through it. It stops short
    # of the roof by nothing -- it runs INTO it, so the two fuse.
    boss = pl.rounded_rect(P.MX_STEM_SQ, P.MX_STEM_SQ, 0.8, seg=8)
    cross = _cross(P.MX_CROSS_A + 0.15, P.MX_CROSS_B + 0.15)
    m += pl.banded(boss, 0.0, P.KEYCAP_SOCKET,
                   [(cross, -OVL, P.KEYCAP_SOCKET + OVL)])

    return [("keycap", m, P.COLORS["keycap"])]


if __name__ == "__main__":
    ok = True
    for n, m, _c in build():
        r = pl.validate(m)
        fit, d = pl.fits_build_plate(m)
        print(f"{n}: shells={r['shells']} watertight={r['watertight']} fits={fit} "
              f"bbox={tuple(round(float(v),1) for v in d)} {r['problems'][:3]}")
        ok &= r["watertight"] and fit
    sys.exit(0 if ok else 1)

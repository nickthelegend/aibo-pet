"""part_horn.py — printed servo couplers. The bits that make the servo
actually drive the arm.

A true 25-tooth involute spline at the MG996R's ~5.92 mm OD works out to
roughly 0.37 mm per tooth flank. That is under a 0.4 mm nozzle, so it cannot
print -- anyone telling you otherwise has not sliced it. Instead the bore is
a root circle with SPLINE_N round-bottomed SCALLOPS that the metal teeth seat
into. 0.9 mm features print cleanly, the shaft self-centres, and the flanks
still key it in rotation.

Two options per servo, because the printed spline is the one part here whose
tolerance you genuinely cannot know until you test it:

  horn-mg996r / horn-sg90    printed cross horn, scalloped spline bore.
                             Drops straight into the yoke's cross recess.
  horn-adapter               belt and braces: a cross plate with a round
                             pocket that captures the STOCK metal horn.
                             Screw the metal horn in, drop the plate into
                             the same recess. Zero printed-spline risk.
  spline-test                three bores at -0.1 / 0 / +0.1 clearance.
                             Print this first. It takes four minutes.

Either way the TORQUE ends up in the yoke's cross slot, not in the screws --
so a loose-ish spline costs you backlash, not a stripped joint.

All print flat, hub up. No supports.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
from shapely import affinity
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import partlib as pl

OVL = pl.OVL


def spline_bore(clear=0.0):
    """Root circle + SPLINE_N scallops. The metal teeth seat in the scallops."""
    root = pl.circle(P.SPLINE_ROOT_D + P.SPLINE_CLEAR + clear, 64)
    r = P.SPLINE_ROOT_D / 2.0
    scallops = [affinity.translate(pl.circle(P.SCALLOP_D + clear, 12),
                                   r * math.cos(a), r * math.sin(a))
                for a in np.linspace(0, 2 * math.pi, P.SPLINE_N, endpoint=False)]
    return unary_union([root] + scallops)


def cross(half, w, hub_d):
    """The profile the yokes' recesses are cut for."""
    return unary_union([
        pl.rounded_rect(2 * half, w, w / 2 - 0.1),
        pl.rounded_rect(w, 2 * half, w / 2 - 0.1),
        pl.circle(hub_d, 48),
    ])


def _horn(half, aw, t, hub_d, hub_t, bore):
    """Plain coupler: cross arms + hub, one bore straight through."""
    m = pl.Mesh()
    m += pl.prism(cross(half, aw, hub_d).difference(bore), 0.0, t)
    m += pl.prism(pl.circle(hub_d, 48).difference(bore), t - OVL, hub_t)
    return m


def _horn_stepped(half, aw, t, hub_d, bore):
    """MG996R coupler with the stepped bore the retaining screw needs:

        0        .. CB_H     head counterbore
        CB_H     .. t        M3 shank -- the shoulder the head clamps against
        t        .. t+SPLINE spline, hub protruding toward the servo

    Total hub protrusion stays under the shaft's ~4.7 mm of exposed spline so
    it grips the teeth rather than bottoming on the servo's boss.
    """
    prof = cross(half, aw, hub_d)
    m = pl.Mesh()
    m += pl.prism(prof.difference(pl.circle(P.HORN_CB_D, 48)), 0.0, P.HORN_CB_H)
    m += pl.prism(prof.difference(pl.circle(P.M3_CLEAR, 32)),
                  P.HORN_CB_H - OVL, t)
    m += pl.prism(pl.circle(hub_d, 48).difference(bore), t - OVL,
                  t + P.HORN_SPLINE_L)
    return m


def build():
    out = []
    out.append(("horn-mg996r",
                _horn_stepped(P.HORN_ARM_HALF, P.HORN_ARM_W, P.HORN_T,
                              P.HORN_HUB_D, spline_bore()),
                P.COLORS["metal"]))
    out.append(("horn-sg90",
                _horn(P.SG_HORN_ARM, P.SG_HORN_W, P.SG_HORN_T, P.SG_HORN_HUB_D,
                      P.SG_HORN_HUB_D * 0.7,
                      pl.circle(P.SG_SHAFT_D + 0.25, 48)),
                P.COLORS["metal"]))

    # adapter: cross outline, round pocket capturing a STOCK metal horn
    prof = cross(P.HORN_ARM_HALF, P.HORN_ARM_W, P.HORN_DISC_D + 3.0)
    pocket = pl.circle(P.HORN_DISC_D + 0.4, 64)
    pilots = unary_union([
        affinity.translate(pl.circle(P.M2_PILOT, 16),
                           P.HORN_SCREW_R * math.cos(a), P.HORN_SCREW_R * math.sin(a))
        for a in np.linspace(0, 2 * math.pi, 4, endpoint=False)])
    hubclear = pl.circle(9.0, 48)
    ad = pl.Mesh()
    ad += pl.prism(prof.difference(unary_union([pilots, hubclear])), 0.0, P.HORN_T)
    ad += pl.prism(prof.difference(pocket), P.HORN_T - OVL, P.HORN_T + P.HORN_HUB_T)
    out.append(("horn-adapter", ad, P.COLORS["metal"]))

    # test coupon: three clearances, labelled by notch count
    t = pl.Mesh()
    plate = pl.rounded_rect(66.0, 26.0, 3.0)
    bores = []
    for i, c in enumerate((-0.1, 0.0, 0.1)):
        bores.append(affinity.translate(spline_bore(c), -21.0 + i * 21.0, 0.0))
        for k in range(i + 1):
            bores.append(affinity.translate(pl.rounded_rect(1.6, 3.0, 0.4),
                                            -21.0 + i * 21.0 - 3.0 + k * 3.0, 10.0))
    t += pl.prism(plate.difference(unary_union(bores)), 0.0, 6.0)
    out.append(("spline-test", t, P.COLORS["metal"]))
    return out


if __name__ == "__main__":
    ok = True
    for n, m, _c in build():
        r = pl.validate(m)
        fit, d = pl.fits_build_plate(m)
        print(f"{n:14s} shells={r['shells']:3d} watertight={r['watertight']} "
              f"fits={fit} bbox={tuple(round(float(v),1) for v in d)} {r['problems'][:2]}")
        ok &= r["watertight"] and fit
    print(f"  spline: {P.SPLINE_N} scallops of {P.SCALLOP_D} at r="
          f"{P.SPLINE_ROOT_D/2:.2f}, root bore {P.SPLINE_ROOT_D + P.SPLINE_CLEAR:.2f}")
    sys.exit(0 if ok else 1)

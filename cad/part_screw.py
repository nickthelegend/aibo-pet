"""part_screw.py — the printed retaining screw for the arm yokes.

Why it exists: the yoke had nothing holding it on. Torque was always carried
by the drive plate's cross recess, which is fine, but AXIALLY the segment was
held by 3.0 mm of stub-axle engagement and friction. Spread the two plates
3 mm and the whole arm lifts off. On the base joint gravity hides it; on the
elbow the fore-arm hangs sideways off it at full reach.

This is the keeper, and it prints -- no metal.

  M6 x 2.0 trapezoidal, NOT a real metric profile. A real M3 is 0.5 mm of
  pitch, which is roughly a quarter of a 0.4 nozzle per flank: it does not
  survive slicing. 2.0 mm of pitch is 10 layers a turn at 0.2 mm and comes
  out as something you can actually run down a hole. Flanks sit near 45
  degrees so the form is self-supporting where the thread ends up horizontal.

  Nominal on the screw, clearance cut into the BORE (SCREW_FIT), so this part
  is the reference and the hole is the one that moves.

Prints standing on its head, thread pointing UP. The head is the widest part
and goes on the bed; the thread is a vertical helix, which is the one
orientation where a printed thread is genuinely clean. No supports.

The knurl is not decoration -- this is hand-tightened, there is no hex on it.
"""
from __future__ import annotations

import math
import os
import sys

from shapely import affinity
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import partlib as pl

OVL = pl.OVL


def _head():
    """Round head with a scalloped knurl so it can be turned by hand."""
    disc = pl.circle(P.SCREW_HEAD_D, 64)
    r = P.SCREW_HEAD_D / 2.0
    cuts = [affinity.translate(pl.circle(3.2, 16),
                               r * math.cos(2 * math.pi * i / P.SCREW_KNURL_N),
                               r * math.sin(2 * math.pi * i / P.SCREW_KNURL_N))
            for i in range(P.SCREW_KNURL_N)]
    return disc.difference(unary_union(cuts))


def build():
    m = pl.Mesh()
    # head on the bed, 0 .. SCREW_HEAD_T
    m += pl.prism(_head(), 0.0, P.SCREW_HEAD_T)
    # a plain collar under the thread: the first turn of any printed thread is
    # ragged, and this keeps that off the bearing face
    m += pl.prism(pl.circle(P.SCREW_MAJOR - 2 * 0.54 * P.SCREW_PITCH, 48),
                  P.SCREW_HEAD_T - OVL, P.SCREW_HEAD_T + 1.0)
    # thread, nominal
    m += pl.thread(P.SCREW_MAJOR, P.SCREW_PITCH,
                   P.SCREW_HEAD_T + 1.0 - OVL,
                   P.SCREW_HEAD_T + 1.0 + P.SCREW_ENGAGE)
    return [("yoke-screw", m, P.COLORS["joint"])]


if __name__ == "__main__":
    ok = True
    for n, m, _c in build():
        r = pl.validate(m)
        fit, d = pl.fits_build_plate(m)
        print(f"{n}: shells={r['shells']} watertight={r['watertight']} fits={fit} "
              f"bbox={tuple(round(float(v),1) for v in d)} {r['problems'][:3]}")
        ok &= r["watertight"] and fit
    sys.exit(0 if ok else 1)

"""part_retainers.py — the small parts that positively hold components down.

Foam tape is not a fixing. Both of these screw into posts that are part of
the base, and both print flat with no supports.

  spk-clamp   bar across the top of the speaker pocket. Its screws go into
              the pocket's SIDE RAILS -- there are no clamp posts, because
              posts in the bay are exactly what leaves no room for the
              driver's leads.
  mic-tab     drops over the mic board's slot mouth (belt and braces on top
              of the two moulded retaining lips).
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
XF = P.SPK_CTR_X + P.SPK_T + P.SPK_FIT
Y0 = P.SPK_CTR_Y - P.SPK_L / 2 - P.SPK_FIT
Y1 = P.SPK_CTR_Y + P.SPK_L / 2 + P.SPK_FIT
POST_Y = (Y0 - P.SPK_RAIL_W / 2, Y1 + P.SPK_RAIL_W / 2)   # centres, in the rails


def build():
    bar = pl.rounded_rect(P.SPK_CLAMP_W,
                          (POST_Y[1] - POST_Y[0]) + P.SPK_CLAMP_W, 3.0)
    holes = unary_union([affinity.translate(pl.circle(P.M2_CLEAR), 0.0,
                                            py - (POST_Y[0] + POST_Y[1]) / 2)
                         for py in POST_Y])
    clamp = pl.prism(bar.difference(holes), 0.0, P.SPK_CLAMP_T)

    d = P.MIC_D + P.MIC_FIT
    tab = pl.rounded_rect(d + 6.0, 7.0, 2.0).difference(
        affinity.translate(pl.circle(P.M2_CLEAR), 0.0, 1.6))
    mic = pl.prism(tab, 0.0, 2.0)
    mic += pl.prism(box(-d / 2 + 1.0, -3.5, d / 2 - 1.0, -1.6), 2.0 - OVL, 2.0 + 1.2)
    return [("spk-clamp", clamp, P.COLORS["joint"]),
            ("mic-tab", mic, P.COLORS["joint"])]


if __name__ == "__main__":
    ok = True
    for n, m, _c in build():
        r = pl.validate(m)
        print(f"{n:10s} shells={r['shells']} watertight={r['watertight']} "
              f"bbox={tuple(round(float(v),1) for v in pl.fits_build_plate(m)[1])} "
              f"{r['problems'][:2]}")
        ok &= r["watertight"]
    sys.exit(0 if ok else 1)

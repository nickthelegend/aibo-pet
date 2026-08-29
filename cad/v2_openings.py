"""v2_openings.py — does every opening actually reach DAYLIGHT?

A hole in the wall is not a hole in the lamp. Grounding the rear crown to
the tub wall filled the annulus r72..81.2 all the way round, and every
opening in that wall -- the speaker grille, the USB charge well, the mic
port, the MX lead window -- ended up bricked up behind it. The wall had a
hole; the crown behind the wall did not; from outside there was no hole at
all. The user found three of them by trying to thread a wire.

So this fires a ray straight out from each opening's own station and asks
whether it escapes the model. Not "is the wall cut" -- is there a path to
the outside world.

    .venv/bin/python cad/v2_openings.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import solidtest as ST
import v2_parts as V

R0, R1 = 76.0, 94.0          # from just outside the wall to open air


def main():
    tub = {n: m for n, m, _c in V.tub()}["v2-tub"]
    usb_z = V.FLOOR + V.ESP_POST + 1.4 - 2.4
    # (label, angle, heights to try -- an opening counts as open if ANY
    #  height in its own band gets a clear ray out)
    STATIONS = [
        ("speaker grille, 0 deg", 0.0, [10.0, 16.0, 22.0, 28.0]),
        ("speaker grille, 10 deg", 10.0, [10.0, 16.0, 22.0, 28.0]),
        ("speaker grille, -30 deg", -30.0, [10.0, 16.0, 22.0, 28.0]),
        ("intake grille, 155 deg", 155.0, [10.0, 16.0, 22.0, 28.0]),
        ("USB charge well", 30.0, [usb_z + 2, usb_z + 4, usb_z + 6]),
        ("mic port", 180.0, [26.6, 27.0, 27.4, 27.8]),
        ("MX lead window", 270.0, [27.0, 29.0, 31.0, 33.0]),
    ]
    print("HOTARU 2.0 -- every opening must reach daylight")
    print("=" * 60)
    fails = []
    for label, ang, zs in STATIONS:
        best = None
        # Jitter the ray off the seam. A Ø2.5 port probed exactly on its own
        # axis puts every sample on the mesh's polar seam, where ray parity
        # is a coin flip -- this check called the mic port open on one run
        # and bricked on the next before the jitter went in. The repo has
        # learned this lesson twice now.
        for z in zs:
            for da, dz in ((0.0, 0.0), (0.6, 0.0), (-0.6, 0.0),
                           (0.0, 0.35), (0.0, -0.35), (0.35, 0.2)):
                rad = math.radians(ang + da)
                pts = np.array([[r * math.cos(rad), r * math.sin(rad), z + dz]
                                for r in np.arange(R0, R1, 0.75)])
                if not ST.inside(tub, pts).any():
                    best = z + dz
                    break
            if best is not None:
                break
        ok = best is not None
        if not ok:
            fails.append(f"{label}: no clear path out at any of {zs}")
        print(f"  {'OPEN   ' if ok else 'BRICKED'} {label:26s}"
              + (f" at z={best:.1f}" if ok else ""))
    print("=" * 60)
    if fails:
        print(f"{len(fails)} opening(s) do not reach the outside:")
        for f in fails:
            print("  - " + f)
        return 1
    print("every opening reaches daylight")
    return 0


if __name__ == "__main__":
    sys.exit(main())

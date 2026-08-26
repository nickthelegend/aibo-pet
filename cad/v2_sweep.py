"""v2_sweep.py — does the disc actually TURN?

Every other audit checks the robot standing still. That answers "do the
parts fit", not "does the mechanism work". This one rotates the moving
assembly -- disc, tower, both links, head, shade, screws, and the three
servos that ride with them -- through the full pan range in steps, and at
every step checks it against everything bolted to the tub.

If any step collides, the disc does not turn 180 degrees, no matter how
cleanly the static model assembles.

    .venv/bin/python cad/v2_sweep.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import solidtest as ST
import v2_assembly as A
import v2_parts as V

# Everything that turns with the disc. The pan servo does NOT: its case is
# bolted to the tub floor and only its output shaft turns.
STATIC_PREFIX = ("v2-tub", "mx-", "pan-")
STEP_DEG = 7.5
RANGE_DEG = 180.0


def split():
    moving, static = [], []
    for n, m, _c in A.world_items():
        (static if n.startswith(STATIC_PREFIX) else moving).append((n, m))
    return moving, static


def main():
    moving, static = split()
    print("HOTARU 2.0 -- pan sweep")
    print("=" * 62)
    print(f"moving with the disc : {len(moving)} parts")
    print(f"fixed to the tub     : {len(static)} parts")
    print(f"sweep                : +/-{RANGE_DEG/2:.0f} deg in {STEP_DEG} deg steps\n")

    rng = np.random.default_rng(17)
    worst = None
    bad = []
    for k in range(int(RANGE_DEG / STEP_DEG) + 1):
        a = -RANGE_DEG / 2 + k * STEP_DEG
        hits = 0
        detail = []
        for mn, mm in moving:
            q = mm.copy()
            q.rotate_z(a)
            bq = q.bounds()
            for sn, sm in static:
                bs = sm.bounds()
                if not all(min(bq[i+3], bs[i+3]) - max(bq[i], bs[i]) > 0.05
                           for i in range(3)):
                    continue
                lo = [max(bq[i], bs[i]) for i in range(3)]
                hi = [min(bq[i+3], bs[i+3]) for i in range(3)]
                pts = rng.uniform(lo, hi, size=(1400, 3))
                n_in = int((ST.inside(q, pts) & ST.inside(sm, pts)).sum())
                if n_in > 5:
                    hits += n_in
                    detail.append(f"{mn}/{sn}={n_in}")
        flag = "clear" if not hits else "COLLIDES  " + ", ".join(detail[:3])
        if hits:
            bad.append(a)
        print(f"  {a:+7.1f} deg   {flag}")
    print("=" * 62)
    if bad:
        print(f"the disc CANNOT complete the sweep: {len(bad)} blocked step(s) "
              f"at {[round(x,1) for x in bad[:8]]}")
        return 1
    print(f"the disc turns the full {RANGE_DEG:.0f} degrees without touching "
          f"anything fixed to the tub")
    return 0


if __name__ == "__main__":
    sys.exit(main())

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
# bolted to the tub floor and only its output shaft turns. Neither do the
# BOARDS -- they sit on the tub's own standoffs, and leaving them out of
# this list swept the ESP32 and the mic round the inside of the tub and
# reported the disc as seized on its own electronics.
STATIC_PREFIX = ("v2-tub", "mx-", "v2-keycap", "pan-",
                 "esp32-", "amp-", "mic-", "spk-")
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
    bad = []

    def touching(a):
        out = {}
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
                    out[(mn, sn)] = n_in
        return out

    # The pan horn is CLAMPED to the pan servo's output: it overlaps that
    # servo's spline and boss at every angle, by design, because that IS the
    # drive. That one coupling is exempt.
    #
    # Deliberately NOT a blanket "whatever touches at 0 degrees" baseline. The
    # defect this audit was written for -- the disc sitting on the MG996R's
    # secondary case hump -- is present at 0 degrees too, so baselining the
    # built pose would have hidden the very thing it caught. Everything except
    # the horn/servo coupling must be clear at every angle, 0 included.
    baseline = {(m, s) for m, s in touching(0.0)
                if m == "horn-pan" and s.startswith("pan-")}
    print("  designed coupling held out of the sweep: "
          + ", ".join(sorted(f"{m}/{s}" for m, s in baseline)))

    for k in range(int(RANGE_DEG / STEP_DEG) + 1):
        a = -RANGE_DEG / 2 + k * STEP_DEG
        hit = {p: n for p, n in touching(a).items() if p not in baseline}
        detail = [f"{m}/{s}={n}" for (m, s), n in hit.items()]
        flag = "clear" if not hit else "COLLIDES  " + ", ".join(detail[:3])
        if hit:
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

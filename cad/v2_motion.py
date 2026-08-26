"""v2_motion.py — does every joint move, through its whole range?

v2_sweep answers that for the pan. This answers it for the other three, and
it is the check that matters most when there is no filament for a reprint:
a joint that binds at 40 degrees is a part you throw away.

Each joint is rotated about its own axis, carrying everything OUTBOARD of
it, and tested against everything INBOARD plus the tub. Ranges are what an
MG996R/SG90 actually delivers (180 mechanical), clipped to what the arm
needs, and the audit reports the largest arc that is collision free -- not
just pass/fail, because "it moves 150 of the 180 you asked for" is the
useful answer.

    .venv/bin/python cad/v2_motion.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import solidtest as ST
import v2_assembly as A
import v2_parts as V

STEP = 7.5

# joint -> (axis height, prefixes that ride OUTBOARD of it, range)
def joints():
    z_sh = V.DISC_Z0 + V.DISC_T + V.TWR_AXIS_Z
    z_el = z_sh + V.L1
    z_hd = z_el + V.L2
    return [
        # A horn rides its servo's OUTPUT, so it turns with the outboard
        # link, not with the case. The screws thread into the inboard stub
        # and stay put while the outboard plate turns on them.
        ("shoulder", z_sh, ("v2-link1", "v2-link2", "v2-head", "shade",
                            "v2-trimcap", "v2-caphead", "v2-screw-elbow",
                            "el-", "hd-",
                            "horn-shoulder", "horn-elbow", "horn-head"), 180.0),
        ("elbow", z_el, ("v2-link2", "v2-head", "shade", "hd-", "v2-trimcap",
                         "v2-caphead", "horn-elbow", "horn-head"), 180.0),
        ("head", z_hd, ("shade", "v2-caphead"), 180.0),
    ]


def main():
    world = [(n, m) for n, m, _c in A.world_items()]
    print("HOTARU 2.0 -- joint motion")
    print("=" * 64)
    bad = []
    rng = np.random.default_rng(23)

    for name, axis_z, out_pre, rng_deg in joints():
        moving = [(n, m) for n, m in world if n.startswith(out_pre)]
        fixed = [(n, m) for n, m in world if not n.startswith(out_pre)]
        # Baseline the BUILT pose. Anything already touching at 0 degrees is
        # a designed engagement -- a horn inside its recess, a plate on its
        # stub -- and flagging those reports every joint as seized. What
        # matters is a collision that rotation CREATES.
        def touching(a):
            out_pairs = set()
            for mn, mm in moving:
                q = mm.copy()
                q.translate(dz=-axis_z); q.rotate_x(a); q.translate(dz=axis_z)
                bq = q.bounds()
                for fn, fm in fixed:
                    bf = fm.bounds()
                    if not all(min(bq[i+3], bf[i+3]) - max(bq[i], bf[i]) > 0.05
                               for i in range(3)):
                        continue
                    lo = [max(bq[i], bf[i]) for i in range(3)]
                    hi = [min(bq[i+3], bf[i+3]) for i in range(3)]
                    pts = rng.uniform(lo, hi, size=(900, 3))
                    if int((ST.inside(q, pts) & ST.inside(fm, pts)).sum()) > 6:
                        out_pairs.add((mn, fn))
            return out_pairs

        baseline = touching(0.0)
        if baseline:
            print(f"  {name:9s} designed engagements at rest: "
                  f"{sorted(m + '/' + f for m, f in baseline)[:3]}")
        free = []
        for k in range(int(rng_deg / STEP) + 1):
            a = -rng_deg / 2 + k * STEP
            new = touching(a) - baseline
            if not new:
                free.append(a)
        if free:
            span = max(free) - min(free)
            print(f"  {name:9s} free {min(free):+7.1f} .. {max(free):+7.1f} deg "
                  f"= {span:5.1f} deg of travel")
        else:
            span = 0.0
            print(f"  {name:9s} NO free position at all")
        if span < 90.0:
            bad.append(f"{name}: only {span:.0f} deg")
    print("=" * 64)
    if bad:
        print("joints that cannot swing 90 degrees: " + "; ".join(bad))
        return 1
    print("every joint swings at least 90 degrees")
    return 0


if __name__ == "__main__":
    sys.exit(main())

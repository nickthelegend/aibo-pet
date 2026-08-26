"""Two openings that ABUT in z must not plug each other.

A pin relief whose ceiling is a pocket's floor is the real case: the mic
cradle. banded() stretches an opening band OVL into its neighbours so the
shells fuse; unmasked, that stretch lays the lower band's solid straight
across the upper opening's floor.

This is a check, so it carries its own proof: OLD reproduces the plug, NEW
does not. A test that has never been seen to fail is not evidence.
"""
import sys, os
import numpy as np
from shapely.geometry import box as _sbox
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import partlib as pl
import solidtest as ST

OUTER = _sbox(-10, -10, 10, 10)
POCK = _sbox(-8, -8, 8, 8)          # upper opening, floor at z=10
PIN = _sbox(-3, -3, 3, 3)           # lower opening, ceiling at z=10
OPEN = [(POCK, 10.0, 20.0), (PIN, 2.0, 10.0)]

# sample the pocket floor just above z=10, outside the pin bore: the pocket
# is open there, so nothing may be solid
rng = np.random.default_rng(0)
P = rng.uniform([-8, -8, 10.02], [8, 8, 10.18], size=(4000, 3))
P = P[(np.abs(P[:, 0]) > 3.2) | (np.abs(P[:, 1]) > 3.2)]


def plugged(fn):
    return int(ST.inside(fn(OUTER, 0.0, 20.0, OPEN), P).sum())


def _old(profile, z0, z1, openings):
    marks = {z0, z1}
    for _g, a, b in openings:
        if z0 < a < z1: marks.add(a)
        if z0 < b < z1: marks.add(b)
    m = pl.Mesh()
    zs = sorted(marks)
    for a, b in zip(zs[:-1], zs[1:]):
        if b - a < 1e-6: continue
        act = [g for g, oa, ob in openings if oa <= a + 1e-6 and ob >= b - 1e-6]
        if act:
            cut = profile.difference(unary_union(act))
            if not cut.is_empty:
                m += pl.prism(cut, a - pl.OVL if a > z0 else a,
                              b + pl.OVL if b < z1 else b)
        else:
            m += pl.prism(profile, a, b)
    return m


if __name__ == "__main__":
    old, new = plugged(_old), plugged(pl.banded)
    print(f"  probes above the pocket floor, outside the pin bore: {len(P)}")
    print(f"  OLD banded  solid at {old:5d}   <- the plug this check exists for")
    print(f"  NEW banded  solid at {new:5d}")
    ok = old > 0 and new == 0
    print("PASS" if ok else "FAIL", "-- abutting openings stay open")
    sys.exit(0 if ok else 1)

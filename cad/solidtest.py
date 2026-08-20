"""solidtest.py — a point-in-solid test that survives overlapping shells.

partlib's parity test casts one ray and counts crossings against EVERY
triangle in the mesh. That is correct for a single closed shell and wrong
here, because a part in this repo is a UNION OF OVERLAPPING SHELLS: a point
inside two of them crosses four surfaces, comes back even, and reports as
outside. I used it to tell the user the base and shoulder did not interfere.
That conclusion was unsupported.

A point is inside a union if it is inside ANY member. So split the mesh into
connected shells first, run parity per shell, and OR. Each shell is closed and
manifold on its own, which is exactly the condition parity needs.

    .venv/bin/python cad/solidtest.py        # self test
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import audit_printable as ap


def shells(mesh):
    """[(A, B, C)] one entry per connected shell, as triangle vertex arrays."""
    V, F = mesh._np()
    parent = np.arange(len(V))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for tri in F:
        r0 = find(tri[0])
        for k in (1, 2):
            r1 = find(tri[k])
            if r0 != r1:
                parent[r1] = r0
    roots = np.array([find(t) for t in F[:, 0]])
    out = []
    for rt in np.unique(roots):
        f = F[roots == rt]
        out.append((V[f[:, 0]], V[f[:, 1]], V[f[:, 2]]))
    return out


def inside(mesh, pts):
    """Boolean per point: inside the UNION of the mesh's shells."""
    acc = np.zeros(len(pts), dtype=bool)
    for A, B, C in shells(mesh):
        acc |= ap._inside(pts, A, B, C)
    return acc


def _self_test():
    """Prove it: two overlapping boxes. A point in the overlap is inside the
    union, and the naive whole-mesh parity test says it is not."""
    import partlib as pl
    from shapely.geometry import box as _b
    m = pl.prism(_b(0, 0, 10, 10), 0, 10)
    m += pl.prism(_b(5, 0, 15, 10), 0, 10)     # overlaps 5..10 in X
    p = np.array([[7.5, 5.0, 5.0]])            # squarely in the overlap
    V, F = m._np()
    naive = ap._inside(p, V[F[:, 0]], V[F[:, 1]], V[F[:, 2]])[0]
    fixed = inside(m, p)[0]
    print(f"point inside BOTH boxes -> naive parity: {naive}   per shell: {fixed}")
    ok = (not naive) and fixed
    print("naive test is broken exactly as described" if ok
          else "unexpected: re-examine before trusting either")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(_self_test())

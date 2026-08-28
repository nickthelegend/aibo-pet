"""v2_onepiece.py — is each printed part actually ONE piece?

pl.validate() reports watertight, and a part in two separate islands is
still perfectly watertight: each island is a closed surface. So the tub
passed every audit in this repo while its rear crown and button turret
floated 3.2 mm clear of the wall at every height, joined to nothing. It
printed, it came off the plate in two pieces, and the user found it by
holding it -- which is the worst way to find it.

This voxelises each printed part and flood-fills the solid cells. One
region means one piece. More than one means the slicer will hand you a
bag of parts, however watertight each of them is.

    .venv/bin/python cad/v2_onepiece.py
"""
from __future__ import annotations

import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import audit_printable as ap
import solidtest as ST
import v2_assembly as A

# Parts that are deliberately a TRAY of pieces on one plate -- bolts,
# clamp bars, horns. They are meant to come off separate.
MULTI = {
    "v2-clamps": 6, "v2-bolts": 22, "v2-horns": 4, "v2-screws": 2,
    "v2-disckeys": 2, "v2-link1-spacers": 2, "v2-link2-spacers": 3,
    "v2-link1-ledges": 2,
}


def islands(mesh):
    """(pieces, cell counts) by SHELL ADJACENCY, not voxels.

    A first cut voxelised at 2.5 mm and called the cone 188 pieces: a
    1.6 mm wall simply falls between 2.5 mm samples, so thin geometry
    shatters and the check cries wolf on everything. These parts are unions
    of overlapping shells, so the honest test is which shells actually
    touch: build a graph with an edge wherever two shells interpenetrate,
    and count its connected components.
    """
    shells = ST.shells(mesh)
    n = len(shells)
    verts = [np.vstack([A_, B_, C_]) for A_, B_, C_ in shells]
    boxes = [(v.min(axis=0), v.max(axis=0)) for v in verts]
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i in range(n):
        for j in range(i + 1, n):
            if find(i) == find(j):
                continue
            lo_i, hi_i = boxes[i]
            lo_j, hi_j = boxes[j]
            if np.any(hi_i < lo_j - 1e-6) or np.any(hi_j < lo_i - 1e-6):
                continue                      # bboxes miss: cannot touch
            a, b = shells[i], shells[j]
            vi = verts[i]
            if len(vi) > 400:
                vi = vi[:: max(1, len(vi) // 400)]
            vj = verts[j]
            if len(vj) > 400:
                vj = vj[:: max(1, len(vj) // 400)]
            touch = (ap._inside(vi, *b).any() or ap._inside(vj, *a).any())
            if touch:
                parent[find(i)] = find(j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    sizes = sorted((sum(len(shells[k][0]) for k in g) for g in groups.values()),
                   reverse=True)
    return len(sizes), sizes


def main():
    print("HOTARU 2.0 -- is every printed part one piece?")
    print("=" * 62)
    print(f"{'part':18s} {'pieces':>7s}   triangles per piece")
    fails = []
    for n, m in A.print_items():
        k, sizes = islands(m)
        want = MULTI.get(n, 1)
        ok = k == want
        note = f"  (a tray of {want})" if want > 1 else ""
        if not ok:
            fails.append(f"{n}: {k} pieces, expected {want}  {sizes[:4]}")
        print(f"  {'ok  ' if ok else 'FAIL'} {n:18s} {k:5d}   "
              f"{sizes[:4]}{note}")
    print("=" * 62)
    if fails:
        print(f"{len(fails)} part(s) would come off the plate in pieces:")
        for f in fails:
            print("  - " + f)
        return 1
    print("every printed part is a single connected solid")
    return 0


if __name__ == "__main__":
    sys.exit(main())

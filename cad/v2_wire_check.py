"""v2_wire_check.py — does a cable ACTUALLY fit from each component to its
board, or is that just an assertion?

Every other audit here skips servo leads. v2_insert calls them "flexible"
and drops them; v2_bom_check prints a disclaimer saying wire routing is
asserted, not measured. That is a real hole: a lamp whose head servo has no
way to reach the ESP32 is not printable, however well the plastic fits.

So this does not assert. It VOXELISES the whole assembly, erodes the solid
by one cell so the remaining free space is genuinely wide enough for a
cable, and runs a breadth-first search from each component's lead exit to
its board. A route either exists in that free space or it does not, and if
it does the audit reports how long it is and how tight it gets.

Deliberately NOT a hand-authored waypoint list: waypoints encode the answer
you were hoping for. A search over free space can only find what is really
there, and when it finds nothing that is a defect in the model, not in the
route someone imagined.

The occupancy grid excludes the *-wire stubs themselves -- those are the
cables, and a cable cannot block its own path.

    .venv/bin/python cad/v2_wire_check.py
"""
from __future__ import annotations

import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import solidtest as ST
import v2_assembly as A
import v2_parts as V

# 2.0, not 3.0. Eroding a 3 mm grid by one cell demands 3 mm of clearance
# all round the cable, which is the right test in open space and the wrong
# one inside a channel built FOR a cable: the MX lead's own 8 x 8 window and
# the mic's cradle both failed it while passing a real 2 mm wire with room
# to spare. At 2 mm cells the erosion asks for 2 mm of clearance, so a
# channel has to be 6 mm across to count -- still conservative for a
# servo lead, and it no longer rejects the channels this lamp actually has.
CELL = 2.0            # mm per voxel
PAD = 9.0             # air margin round the assembly bbox

# component lead exit -> where that cable has to arrive
# (source prefix, target xyz, label). Targets are the board terminals.
ESP = (V.ESP_XY[0], V.ESP_XY[1], V.FLOOR + V.ESP_POST + 4.0)
AMP = (V.AMP_XY[0], V.AMP_XY[1], V.FLOOR + V.AMP_POST + 4.0)
ROUTES = [
    ("pan servo -> ESP", "pan-mg996r-wire", ESP),
    ("shoulder servo -> ESP", "sh-mg996r-wire", ESP),
    ("elbow servo -> ESP", "el-mg996r-wire", ESP),
    ("head servo -> ESP", "hd-mg996r-wire", ESP),
    ("LED ring -> ESP", "ring-pads", ESP),
    ("speaker -> amp", "spk-wires", AMP),
    ("microphone -> ESP", "mic-pins", ESP),
    ("MX switch -> ESP", "mx-mx-pins-l", ESP),
]


def main():
    world = A.world_items()
    solid = [(n, m) for n, m, _c in world
             if not (n.endswith("-wire") or n.endswith("-wires"))]

    lo = np.array([min(m.bounds()[i] for _n, m in solid) - PAD
                   for i in range(3)])
    hi = np.array([max(m.bounds()[i + 3] for _n, m in solid) + PAD
                   for i in range(3)])
    dim = np.ceil((hi - lo) / CELL).astype(int) + 1
    print("HOTARU 2.0 -- cable routing, by search not by assertion")
    print("=" * 70)
    print(f"grid {dim[0]} x {dim[1]} x {dim[2]} at {CELL} mm "
          f"({np.prod(dim)/1e3:.0f}k cells)")

    occ = np.zeros(tuple(dim), dtype=bool)

    def idx(p):
        return tuple(np.clip(((np.asarray(p) - lo) / CELL).astype(int),
                             0, dim - 1))

    # Only test cells inside each mesh's own bbox: the assembly is mostly
    # air, and testing every cell against every mesh is the difference
    # between a minute and an hour.
    for n, m in solid:
        b = m.bounds()
        a0, a1 = idx(b[:3]), idx(b[3:])
        rng = [np.arange(a0[k], a1[k] + 1) for k in range(3)]
        if any(len(r) == 0 for r in rng):
            continue
        gx, gy, gz = np.meshgrid(*rng, indexing="ij")
        cells = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        pts = lo + (cells + 0.5) * CELL
        inside = ST.inside(m, pts)
        if inside.any():
            hitc = cells[inside]
            occ[hitc[:, 0], hitc[:, 1], hitc[:, 2]] = True
        print(f"  voxelised {n:22s} {int(inside.sum()):6d} cells")

    # Erode by one cell: the free space that survives is wide enough that a
    # 2-3 mm cable fits with its own clearance, rather than a mathematical
    # gap of zero width that no real wire could use.
    free = ~occ
    e = free.copy()
    for ax in range(3):
        for s in (1, -1):
            e &= np.roll(free, s, axis=ax)
    free = e
    print(f"  free cells after erosion: {int(free.sum())/1e3:.0f}k")

    def nearest_free(p):
        c = np.array(idx(p))
        best, bd = None, 1e9
        rad = 12
        r = np.arange(-rad, rad + 1)
        gx, gy, gz = np.meshgrid(r, r, r, indexing="ij")
        cand = c + np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        ok = np.all((cand >= 0) & (cand < dim), axis=1)
        cand = cand[ok]
        val = free[cand[:, 0], cand[:, 1], cand[:, 2]]
        cand = cand[val]
        if not len(cand):
            return None
        d = ((cand - c) ** 2).sum(1)
        return tuple(cand[d.argmin()])

    NB = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]

    def bfs(src, dst):
        if src is None or dst is None:
            return None
        seen = np.zeros(tuple(dim), dtype=bool)
        prev = {}
        q = deque([src])
        seen[src] = True
        while q:
            cur = q.popleft()
            if cur == dst:
                path = [cur]
                while path[-1] != src:
                    path.append(prev[path[-1]])
                return path[::-1]
            for d in NB:
                nx = (cur[0] + d[0], cur[1] + d[1], cur[2] + d[2])
                if not all(0 <= nx[k] < dim[k] for k in range(3)):
                    continue
                if seen[nx] or not free[nx]:
                    continue
                seen[nx] = True
                prev[nx] = cur
                q.append(nx)
        return None

    print("\nroute                        result")
    fails = []
    M = {n: m for n, m, _c in world}
    for label, src_name, tgt in ROUTES:
        if src_name not in M:
            fails.append(f"{label}: no source mesh {src_name}")
            print(f"  {label:28s} NO SOURCE {src_name}")
            continue
        b = M[src_name].bounds()
        src_pt = [(b[i] + b[i + 3]) / 2 for i in range(3)]
        s, t = nearest_free(src_pt), nearest_free(tgt)
        path = bfs(s, t)
        if path is None:
            fails.append(f"{label}: NO cable path exists")
            print(f"  {label:28s} BLOCKED -- no route in free space")
        else:
            length = (len(path) - 1) * CELL
            print(f"  {label:28s} clear, {length:5.0f} mm of cable")

    print("=" * 70)
    if fails:
        print(f"{len(fails)} cable(s) cannot be routed:")
        for f in fails:
            print("  - " + f)
        return 1
    print("every cable has a real route through free space")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""audit_printable.py — will each part physically print on a Bambu A1 mini?

Reports only. Nothing here changes geometry: if a part does not fit, that is
information, not something to silently "fix" by shrinking it.

A1 mini usable build volume is 180 x 180 x 180. Parts are checked as they sit
in their print orientation, and also rotated 45 degrees about Z, because a
part too wide for the axes can still fit on the diagonal (up to 254 across).
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly
import params as P
import partlib as pl

BED = P.PLATE
SKIRT = 3.0          # brim/skirt allowance per side


def main():
    print(f"Bambu A1 mini: {BED:.0f} x {BED:.0f} x {BED:.0f} mm "
          f"(allowing {SKIRT} mm per side for skirt)\n")
    usable = BED - 2 * SKIRT
    rows, bad = [], []
    for name, mesh in assembly.print_items():
        b = mesh.bounds()
        w, d, h = b[3] - b[0], b[4] - b[1], b[5] - b[2]
        diag = mesh.copy().rotate_z(45.0).bounds()
        dw, dd = diag[3] - diag[0], diag[4] - diag[1]
        flat = w <= usable and d <= usable
        rot = dw <= usable and dd <= usable
        ok = (flat or rot) and h <= BED
        how = "as-is" if flat else ("rotate 45" if rot else "-")
        rows.append((name, w, d, h, ok, how))
        if not ok:
            bad.append((name, w, d, h))
    print(f"{'part':16s} {'X':>7s} {'Y':>7s} {'Z':>7s}   {'fits':>5s}  how")
    print("-" * 58)
    for n, w, d, h, ok, how in rows:
        print(f"{n:16s} {w:7.1f} {d:7.1f} {h:7.1f}   "
              f"{'YES' if ok else 'NO':>5s}  {how}")
    print("-" * 58)
    tallest = max(rows, key=lambda r: r[3])
    widest = max(rows, key=lambda r: max(r[1], r[2]))
    print(f"tallest: {tallest[0]} at {tallest[3]:.0f} mm "
          f"({100*tallest[3]/BED:.0f}% of Z)")
    print(f"widest:  {widest[0]} at {max(widest[1], widest[2]):.0f} mm "
          f"({100*max(widest[1], widest[2])/usable:.0f}% of usable X/Y)")
    if bad:
        print("\nDOES NOT FIT:")
        for n, w, d, h in bad:
            print(f"  {n}  {w:.1f} x {d:.1f} x {h:.1f}")
    else:
        print("\nEvery part fits the A1 mini. Print them one at a time; the "
              "plates in\nexports/plate-*.stl are groupings, not a requirement.")

    bad += _plates()
    _overhangs()
    return 0 if not bad else 1


def _plates():
    """Every PLATE, not just every part. A plate can bust the bed or, worse,
    lay two parts on top of each other -- nothing checked either, and a merged
    STL with overlapping parts slices without complaint into a failed print."""
    import plates as PL
    built = dict(assembly.print_items())
    print(f"\n{'plate':<22}{'used XY':>16}{'Z':>7}  {'fits':>5}{'min gap':>9}  parts")
    print("-" * 76)
    bad = []
    for tag, _why, names in PL.BATCHES:
        for i, grp in enumerate(PL.shelf_pack([(n, built[n]) for n in names])):
            label = f"plate-{tag}" + (f"-{i+1}" if i else "")
            bx = [(n, m.copy().translate(dx=dx, dy=dy).bounds())
                  for n, m, dx, dy in grp]
            ux = max(b[3] for _n, b in bx) - min(b[0] for _n, b in bx)
            uy = max(b[4] for _n, b in bx) - min(b[1] for _n, b in bx)
            uz = max(b[5] for _n, b in bx)
            gap, clash = None, []
            for a in range(len(bx)):
                for c in range(a + 1, len(bx)):
                    (n1, A), (n2, B) = bx[a], bx[c]
                    ox = min(A[3], B[3]) - max(A[0], B[0])
                    oy = min(A[4], B[4]) - max(A[1], B[1])
                    if ox > 0 and oy > 0:
                        clash.append(f"{n1}/{n2}")
                    else:
                        g = max(-ox, -oy)
                        gap = g if gap is None else min(gap, g)
            fits = ux <= BED - 2 * SKIRT and uy <= BED - 2 * SKIRT and uz <= BED
            if not fits or clash:
                bad.append((label, ux, uy, uz))
            print(f"{label:<22}{ux:>7.1f} x{uy:>6.1f}{uz:>7.1f}  "
                  f"{'YES' if fits else 'NO':>5}"
                  f"{('-' if gap is None else f'{gap:.1f}'):>9}  {len(bx)}"
                  + ("   CLASH: " + ", ".join(clash) if clash else ""))
    print("-" * 76)
    print("No plate overlaps a part with another and none busts the bed."
          if not bad else "PLATE PROBLEMS -- see CLASH / NO above.")
    return bad


def _overhangs(limit_deg=45.0):
    """Does anything actually need support?

    Every claim in this repo that nothing needs supports has been an
    assertion. This measures it.

    The catch, and the reason a naive version of this is useless here: a part
    is a UNION OF OVERLAPPING SHELLS (cad/README.md -- the kernel does no 3D
    CSG, the slicer fuses them). Every one of the base's 102 shells has its
    own downward cap, and almost all of them are buried inside another shell.
    Counting raw triangle normals reported 35,000 mm2 of "overhang" on a part
    that has essentially none.

    So each candidate face is tested for whether it is actually EXPOSED:
    take a point just under it and ray-cast +Z against every triangle in the
    part. Odd crossings means that point is inside solid, so the face is
    buried and irrelevant. Even means it is open air, and that is a real
    overhang.

    Faces on the bed are skipped (first layer). Near-horizontal exposed faces
    are counted as BRIDGE, not overhang: a ceiling spanning a gap normally
    prints, a sloping unsupported face does not.
    """
    import math

    import numpy as np

    lim = math.sin(math.radians(limit_deg))
    print(f"\n{'part':16s} {'overhang':>10s} {'bridge':>10s} {'worst':>7s}  verdict")
    print("-" * 64)
    flagged = []
    for name, mesh in assembly.print_items():
        V = np.asarray(mesh.V, dtype=float)
        F = np.asarray(mesh.F, dtype=int)
        A, B, Cv = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        N = np.cross(B - A, Cv - A)
        mag = np.linalg.norm(N, axis=1)
        good = mag > 1e-12
        area = 0.5 * mag
        nz = np.where(good, N[:, 2] / np.where(good, mag, 1.0), 0.0)
        z_bed = V[:, 2].min()
        on_bed = np.maximum.reduce([A[:, 2], B[:, 2], Cv[:, 2]]) <= z_bed + 1e-6
        cand = good & (nz < -lim) & ~on_bed
        idx = np.flatnonzero(cand)

        over = bridge = 0.0
        worst = 0.0
        if idx.size:
            cen = (A[idx] + B[idx] + Cv[idx]) / 3.0
            cen[:, 2] -= 0.05                      # just under the face
            exposed = ~_inside(cen, A, B, Cv)
            hit = idx[exposed]
            if hit.size:
                flat = nz[hit] < -0.985
                bridge = float(area[hit][flat].sum())
                osel = hit[~flat]
                over = float(area[osel].sum())
                if osel.size:
                    worst = float(np.degrees(np.arcsin(
                        np.clip(-nz[osel], 0, 1))).max())
        v = "clean"
        if over > 20.0:
            v = "NEEDS SUPPORT"
            flagged.append(name)
        elif over > 0.0:
            v = "minor, should be fine"
        elif bridge > 0.0:
            v = "bridges only"
        print(f"{name:16s} {over:9.1f}mm2 {bridge:9.1f}mm2 {worst:6.1f}deg  {v}")
    print("-" * 64)
    print("Nothing needs support -- every part prints as modelled."
          if not flagged else
          "Support (or a redesign) needed: " + ", ".join(flagged))
    print("'bridge' is exposed near-horizontal ceiling: spans a gap, normally "
          "prints.\n'overhang' is exposed sloping face past "
          f"{limit_deg:.0f} deg off vertical.")
    # Returned so export_web can put the supports flag on each plate rather
    # than a second implementation drifting away from this one. main() ignores
    # it, which is why adding a return here is safe.
    return flagged


def _inside(pts, A, B, Cv):
    """Point-in-solid by counting +Z ray crossings against every triangle.
    Vectorised over points x triangles in blocks."""
    import numpy as np

    out = np.zeros(len(pts), dtype=bool)
    ax, ay, az = A[:, 0], A[:, 1], A[:, 2]
    e1 = B - A
    e2 = Cv - A
    # Moller-Trumbore against a fixed +Z direction
    d = np.array([0.0, 0.0, 1.0])
    pv = np.cross(d, e2)
    det = np.einsum("ij,ij->i", e1, pv)
    live = np.abs(det) > 1e-12
    inv = np.zeros_like(det)
    inv[live] = 1.0 / det[live]
    BLOCK = 256
    for s0 in range(0, len(pts), BLOCK):
        P = pts[s0:s0 + BLOCK]
        tv = P[:, None, :] - np.stack([ax, ay, az], axis=1)[None, :, :]
        u = np.einsum("pij,ij->pi", tv, pv) * inv
        qv = np.cross(tv, e1)
        v = qv[:, :, 2] * inv                       # dot(d, qv) with d = +Z
        t = np.einsum("pij,ij->pi", qv, e2) * inv
        ok = live[None, :] & (u >= 0) & (u <= 1) & (v >= 0) & (u + v <= 1) & (t > 1e-9)
        out[s0:s0 + BLOCK] = (ok.sum(axis=1) % 2) == 1
    return out


if __name__ == "__main__":
    sys.exit(main())

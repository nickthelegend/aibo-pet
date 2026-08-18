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
    return 0 if not bad else 1


def _plates():
    """Every PLATE, not just every part. A plate can bust the bed or, worse,
    lay two parts on top of each other -- nothing checked either until now,
    and a merged STL with overlapping parts slices without complaint into a
    print that fails."""
    import plates as PL
    built = dict(assembly.print_items())
    print(f"\n{'plate':<20}{'used XY':>16}{'Z':>7}  {'fits':>5}{'min gap':>9}  parts")
    print("-" * 74)
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
            print(f"{label:<20}{ux:>7.1f} x{uy:>6.1f}{uz:>7.1f}  "
                  f"{'YES' if fits else 'NO':>5}"
                  f"{('-' if gap is None else f'{gap:.1f}'):>9}  {len(bx)}"
                  + ("   CLASH: " + ", ".join(clash) if clash else ""))
    print("-" * 74)
    print("No plate overlaps a part with another and none busts the bed."
          if not bad else "PLATE PROBLEMS -- see CLASH / NO above.")
    return bad


if __name__ == "__main__":
    sys.exit(main())

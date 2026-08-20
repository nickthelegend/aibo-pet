"""audit_support.py — what is hanging over thin air?

audit_printable classifies any near-horizontal downward face as a "bridge"
and says it "spans a gap, normally prints". That is only true if the face
actually spans BETWEEN two supports. A tab cantilevered into the middle of a
bore is also a near-horizontal downward face, and it prints as drooping
string. The shoulder's four lid lugs were waved through by exactly that
assumption and came off the printer with their screw holes in mid air.

So this asks the question the classifier never did: for every exposed
downward face, is there any material directly beneath it? Cast a ray straight
down from the centre of the triangle. If it never hits the part again, that
face is over air, and no amount of calling it a bridge changes that.

    .venv/bin/python cad/audit_support.py
    .venv/bin/python cad/audit_support.py --part shoulder --flip
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly
import audit_printable as ap

PLATE_TOL = 0.35        # a face this close to the lowest point IS the plate
# Below this an unsupported island is a chamfer or a fillet tangent, a few
# square millimetres that bridge without noticing. Flagging those would put
# SUPPORTS on the 15 minute test coupon for 5.6 mm2, which trains people to
# ignore the flag. Everything real here is over 100.
MIN_AREA = 25.0         # mm2
# Only faces STEEPER than 45 degrees of overhang can fail to hold themselves
# up. A 45 degree cone has nothing directly beneath it either, but each layer
# lands on the perimeter of the one below and prints fine, so including those
# buries the real defects in thousands of harmless triangles.
STEEP = -0.75           # nz below this is past 45 deg of overhang


def unsupported(mesh):
    """(area_mm2, worst_height_mm, n_faces) hanging over nothing."""
    V, F = mesh._np()
    A, B, C = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    n = np.cross(B - A, C - A)
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-12
    area = ln / 2.0
    nz = np.zeros(len(F))
    nz[ok] = n[ok, 2] / ln[ok]

    cen = (A + B + C) / 3.0
    zmin = V[:, 2].min()

    # exposed downward faces, above the plate, reusing the audit's own
    # buried-shell test so overlapping shells do not report false positives
    cand = np.where(ok & (nz < STEEP) & (cen[:, 2] > zmin + PLATE_TOL))[0]
    if not len(cand):
        return 0.0, 0.0, 0
    # Step just under the face before the point in solid test, exactly as
    # _overhangs does. A centroid left sitting on its own triangle casts its
    # ray up through the body it belongs to and comes back odd.
    probe = cen[cand].copy()
    probe[:, 2] -= 0.05
    keep = ap._inside(probe, A, B, C)
    cand = cand[~keep]
    if not len(cand):
        return 0.0, 0.0, 0

    # Ray straight down from each centroid. Anything hit below supports it.
    lo = np.minimum(np.minimum(A, B), C)
    hi = np.maximum(np.maximum(A, B), C)
    free_area, worst, cnt = 0.0, 0.0, 0
    for i in cand:
        p = cen[i]
        box = (lo[:, 0] <= p[0]) & (hi[:, 0] >= p[0]) & \
              (lo[:, 1] <= p[1]) & (hi[:, 1] >= p[1]) & (lo[:, 2] < p[2] - 1e-6)
        j = np.where(box)[0]
        if not len(j):
            hit = False
        else:
            # barycentric test in XY, then require the hit to be below
            a, b, c = A[j], B[j], C[j]
            d = ((b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) +
                 (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1]))
            d = np.where(np.abs(d) < 1e-12, 1e-12, d)
            w0 = ((b[:, 1] - c[:, 1]) * (p[0] - c[:, 0]) +
                  (c[:, 0] - b[:, 0]) * (p[1] - c[:, 1])) / d
            w1 = ((c[:, 1] - a[:, 1]) * (p[0] - c[:, 0]) +
                  (a[:, 0] - c[:, 0]) * (p[1] - c[:, 1])) / d
            w2 = 1.0 - w0 - w1
            ins = (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9)
            zh = w0 * a[:, 2] + w1 * b[:, 2] + w2 * c[:, 2]
            hit = bool(np.any(ins & (zh < p[2] - 1e-6)))
        if not hit:
            free_area += area[i]
            worst = max(worst, p[2] - zmin)
            cnt += 1
    return free_area, worst, cnt


def main():
    flip = "--flip" in sys.argv
    only = None
    if "--part" in sys.argv:
        only = sys.argv[sys.argv.index("--part") + 1]

    print("area hanging over air (ray cast down from every exposed "
          "downward face)\n")
    print(f"{'part':16s} {'over air':>10s} {'faces':>6s} {'highest':>8s}  verdict")
    print("-" * 60)
    bad = []
    for name, mesh in assembly.print_items():
        if only and name != only:
            continue
        m = mesh.copy()
        if flip:
            m.rotate_x(180.0)
        a, w, c = unsupported(m)
        v = "clean"
        if a > MIN_AREA:
            v = "UNSUPPORTED"
            bad.append(name)
        print(f"{name:16s} {a:9.1f}  {c:6d} {w:7.1f}mm  {v}")
    print("-" * 60)
    print("nothing hangs over air" if not bad else
          "over air: " + ", ".join(bad))
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())

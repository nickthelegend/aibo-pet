"""v2_insert.py — can the thing actually be ASSEMBLED?

Fit and motion are not the same question as closure. A plate can clear every
neighbour in the built pose, swing its full arc, and still be impossible to
put there, because the only path to its seat runs through a part that is
already on the robot. v1 taught this the hard way: the lid passed every
static check and could not be dropped in, because the keys necked the bore
below the bead.

So this walks each part in along a straight line, from clear air to its
seat, and asks whether anything is in the way EN ROUTE. Two design notes:

  * It SEARCHES the six axis directions rather than trusting a hand-written
    approach per part. A guessed direction that happens to point into a wall
    reports a fine part as broken -- which it did, for eight parts, until
    the search replaced the guess. What matters is whether SOME straight
    path exists, and the audit reports which one.
  * The seated pose is baselined the way v2_motion baselines a joint. A part
    resting on its seat touches by design; only a clash the approach CREATES
    is a defect.
  * The shade taught the audit's sharpest lesson: with a closed cross
    recess and closed bores it could not reach its seat along ANY path,
    straight or two-segment -- the yoke problem. The part was redesigned
    (drop-in channels through the yoke tip, retention added back as a glued
    hub cap and a stub screw), and now it passes the same straight-line test
    as everything else. When this row fails, change the part, not the test.

A part that fails here is not a part to reprint. It is a part to redesign.

    .venv/bin/python cad/v2_insert.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import solidtest as ST
import v2_assembly as A

TRAVEL = 30.0          # how far back along the approach to start
STEP = 2.0

DIRS = [((0, 0, -1), "down -Z"), ((0, 0, 1), "up +Z"),
        ((-1, 0, 0), "in -X"), ((1, 0, 0), "in +X"),
        ((0, -1, 0), "in -Y"), ((0, 1, 0), "in +Y"),
        # horizontal diagonals: the disc keys enter radially at 210/330
        ((-0.866, -0.5, 0), "in 210deg"), ((0.866, 0.5, 0), "in 30deg"),
        ((0.866, -0.5, 0), "in 330deg"), ((-0.866, 0.5, 0), "in 150deg")]

# Servo lead-outs are flexible cable, modelled as a rigid stub so the wire
# exit is visible. A stub that fouls a channel it is meant to be threaded
# through is a modelling artifact, not an assembly defect.
FLEX = ("-wire",)

# build order: each part's obstacles are the parts fitted before it
TUB = ("v2-tub",)
ELEC = TUB + ("esp32-", "amp-", "mic-", "spk-", "mx-")
PAN = ELEC + ("pan-",)
DISC = PAN + ("horn-pan", "v2-disc")
TOWER = DISC + ("v2-tower", "sh-")
L1 = TOWER + ("horn-shoulder", "v2-link1", "v2-screw", "el-")
L2 = L1 + ("horn-elbow", "v2-link2", "v2-screw-elbow", "hd-")

STEPS = [
    ("esp32-", TUB), ("amp-", TUB), ("spk-", TUB), ("mic-", TUB),
    ("pan-", ELEC), ("horn-pan", PAN), ("v2-disc", PAN),
    # one step per key: they enter along OPPOSITE radials (210 and 330),
    # so a single step covering both can never find one direction that fits
    ("v2-disckey-0", PAN + ("v2-disc",)),
    ("v2-disckey-1", PAN + ("v2-disc",)), ("v2-tower", DISC),
    ("horn-shoulder", TOWER), ("v2-link1-out", TOWER),
    ("horn-elbow", L1), ("v2-link2-out", L1),
    ("v2-head", L2), ("horn-head", L2),
    ("shade", L2 + ("v2-head", "horn-head")),
    ("ring-", L2 + ("v2-head", "horn-head", "shade")),
    ("v2-conecap", L2 + ("v2-head", "horn-head", "shade", "ring-")),
    ("v2-keycap", TUB + ("mx-",)),
]


def main():
    world = [(n, m) for n, m, _c in A.world_items()]
    print("HOTARU 2.0 -- assembly closure")
    print("=" * 70)
    print(f"{'part':17s} {'goes in':10s}  {'clear for':>10s}   verdict")
    rng = np.random.default_rng(101)
    fails = []

    def clashes(moving, obstacles, d, t):
        """obstacles the part is inside of when it is t mm short of its seat.

        d points ALONG the approach, toward the seat, so backing the part off
        is -d. Getting that sign wrong drives every part deeper into the
        thing it rests on and reports the whole robot as unassemblable.
        """
        hit = set()
        for mn, mm in moving:
            q = mm.copy()
            q.translate(dx=-d[0] * t, dy=-d[1] * t, dz=-d[2] * t)
            bq = q.bounds()
            for fn, fm in obstacles:
                bf = fm.bounds()
                if not all(min(bq[i + 3], bf[i + 3]) - max(bq[i], bf[i]) > 0.05
                           for i in range(3)):
                    continue
                lo = [max(bq[i], bf[i]) for i in range(3)]
                hi = [min(bq[i + 3], bf[i + 3]) for i in range(3)]
                pts = rng.uniform(lo, hi, size=(900, 3))
                if int((ST.inside(q, pts) & ST.inside(fm, pts)).sum()) > 6:
                    hit.add((mn, fn))
        return hit

    def reach(moving, obstacles, d, seated):
        """(clear distance along d, what stopped it) -- capped at TRAVEL"""
        t = STEP
        while t <= TRAVEL + 1e-9:
            new = clashes(moving, obstacles, d, t) - seated
            if new:
                return t - STEP, sorted(a + " into " + b for a, b in new)[0]
            t += STEP
        return TRAVEL, ""

    for pre, placed in STEPS:
        # "v2-disc" is a prefix of "v2-disckey-": without the exclusion the
        # disc step dragged both keys along and drove them into the tub.
        moving = [(n, m) for n, m in world
                  if n.startswith(pre) and not n.endswith(FLEX)
                  and not (pre == "v2-disc" and n.startswith("v2-disckey"))]
        obstacles = [(n, m) for n, m in world
                     if n.startswith(placed) and not n.startswith(pre)
                     and not n.endswith(FLEX)]
        if not moving:
            fails.append(f"{pre}: no such part in the assembly")
            print(f"  {pre:17s} {'--':10s}  {'':>10s}   MISSING")
            continue

        seated = clashes(moving, obstacles, (0, 0, 1), 0.0)
        tried = [(reach(moving, obstacles, d, seated), lbl) for d, lbl in DIRS]
        (got, why), lbl = max(tried, key=lambda r: r[0][0])
        if got >= TRAVEL:
            print(f"  {pre:17s} {lbl:10s}  {'30+ mm':>10s}   PASS")
        else:
            fails.append(f"{pre}: best approach {lbl} runs {got:.0f} mm of "
                         f"{TRAVEL:.0f}, then {why}")
            print(f"  {pre:17s} {lbl:10s}  {got:7.0f} mm   FAIL  {why}")

    # ---- does this check have teeth? ----
    # An all-clear from a check that cannot fail is worth nothing. The shade
    # caps the head from above; dragged in SIDEWAYS it has to pass through
    # the link2 plates, so that approach must be caught. If it is not, the
    # PASSes above are meaningless and this exits non-zero regardless.
    shade = [(n, m) for n, m in world if n.startswith("shade")]
    obst = [(n, m) for n, m in world
            if n.startswith(L2 + ("v2-head", "horn-head"))]
    seated = clashes(shade, obst, (0, 0, 1), 0.0)
    caught = reach(shade, obst, (1, 0, 0), seated)[0] < TRAVEL
    print(f"  {'[self-test]':17s} {'in +X':10s}  "
          f"{'blocked' if caught else 'CLEAR':>10s}   "
          f"{'has teeth' if caught else 'CHECK IS BLIND'}")

    print("=" * 70)
    if not caught:
        print("this audit cannot detect a blocked approach -- results void")
        return 1
    if fails:
        print(f"{len(fails)} part(s) cannot be brought to their seat:")
        for f in fails:
            print("  - " + f)
        return 1
    print("every part reaches its seat along a straight approach")
    return 0


if __name__ == "__main__":
    sys.exit(main())

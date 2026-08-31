"""v2_bom_check.py — is every component you actually bought IN there, does
it fit, and is it ENCLOSED?

The other audits answer "do the printed parts fit each other". They do not
answer the three questions an owner asks before printing:

  1. inventory  every component on the BOM present in the world assembly,
                at its real size -- not a placeholder
  2. clearance  how much air is around each one; a 0.0 gap is a part you
                have to file, and a negative gap is one that does not go in
  3. enclosure  is the electronics bay actually CLOSED? Ray-cast straight
                up from each board: if the ray leaves the model without
                hitting anything, that board is open to the sky.

It also states plainly what it CANNOT verify, because two of the audits in
this repo exclude servo leads as "flexible" and a reader deserves to know
that wire routing is measured by v2_wire_check, not asserted here.

    .venv/bin/python cad/v2_bom_check.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import solidtest as ST
import v2_assembly as A
import v2_parts as V

# what the user said they have, and the prefix that proves it is placed
BOM = [
    ("ESP32-S3 dev board", "esp32-", 1),
    ("MAX98357A amplifier", "amp-", 1),
    ("speaker, 40 x 20", "spk-", 1),
    ("INMP441 microphone", "mic-", 1),
    ("WS2812 LED ring, 4 pads", "ring-", 1),
    ("MX switch + printed keycap", "mx-", 1),
    ("MG996R, pan", "pan-", 1),
    ("MG996R, shoulder", "sh-", 1),
    ("MG996R, elbow", "el-", 1),
    ("MG996R, head", "hd-", 1),
]

PRINTED = ("v2-", "shade", "horn-")

# A component INSIDE a printed part is not automatically a fault. Three
# pairs are the machine working, and v2_audit already exempts them:
#   a horn grips its servo's spline -- that IS the drive
#   a keycap presses onto the switch stem -- that IS the button
# Anything not listed here is a real interference.
BY_DESIGN = {
    ("pan-", "horn-pan"), ("sh-", "horn-shoulder"),
    ("el-", "horn-elbow"), ("hd-", "horn-head"),
    ("mx-", "v2-keycap"),
}


def main():
    world = A.world_items()
    M = {n: m for n, m, _c in world}
    printed = [(n, m) for n, m, _c in world if n.startswith(PRINTED)]
    fails, warns = [], []

    print("HOTARU 2.0 -- bill of materials, fit and enclosure")
    print("=" * 68)

    # ---- 1. inventory ----
    print("\ncomponent present, and how big it really is")
    for label, pre, want in BOM:
        parts = [(n, m) for n, m in M.items() if n.startswith(pre)]
        if len(parts) < want:
            fails.append(f"{label}: MISSING from the assembly")
            print(f"  FAIL {label:30s} missing")
            continue
        lo = [min(m.bounds()[i] for _n, m in parts) for i in range(3)]
        hi = [max(m.bounds()[i + 3] for _n, m in parts) for i in range(3)]
        d = [hi[i] - lo[i] for i in range(3)]
        print(f"  ok   {label:30s} {len(parts):2d} meshes  "
              f"{d[0]:5.1f} x {d[1]:5.1f} x {d[2]:5.1f} mm")

    # ---- 2. clearance to the nearest printed wall ----
    # Sample the component's own surface and march each point outward; the
    # honest measure is how far a point can move before it is inside
    # something printed. Cheaper and just as decisive: nearest-vertex gap.
    print("\nclearance from each component to the nearest printed part")
    rng = np.random.default_rng(5)
    for label, pre, _w in BOM:
        parts = [(n, m) for n, m in M.items() if n.startswith(pre)]
        # shrink PER MESH, not per component: the MX "component" is a
        # switch plus a keycap sitting above it, and shrinking toward their
        # COMBINED centre pushed switch vertices up into the keycap's
        # neighbourhood and reported 44 phantom burials.
        chunks = []
        for _n, m in parts:
            # A SPLINE's entire purpose is to enter the part it drives, so
            # it is excluded here rather than reported as a burial. This is
            # not an unchecked hole: v2_audit's engagement table measures
            # the spline's reach into the horn socket as a NUMBER, which is
            # the check that actually matters for a drive.
            if _n.endswith("-spline"):
                continue
            v = np.asarray(m.V)
            c = v.mean(axis=0)
            chunks.append(v + (c - v) * (0.15 / np.maximum(
                np.linalg.norm(c - v, axis=1, keepdims=True), 1e-6)))
        pts = np.vstack(chunks)
        # Pull each vertex 0.15 toward the component's own centre before
        # testing. A component that RESTS on a printed face shares that
        # surface, and a raw vertex sitting on it reads as "inside" by
        # parity -- which is how a servo tab lying on its ledge, exactly as
        # designed, was being reported as buried. Shrinking keeps real
        # burials (they survive 0.15) and drops surface kisses. Cheaper and
        # more honest than an exemption list that grows every time a
        # contact is found.
        if len(pts) > 1500:
            pts = pts[rng.choice(len(pts), 1500, replace=False)]
        worst, who = 1e9, ""
        buried = 0
        for pn, pm in printed:
            b = pm.bounds()
            near = pts[(pts[:, 0] > b[0] - 6) & (pts[:, 0] < b[3] + 6) &
                       (pts[:, 1] > b[1] - 6) & (pts[:, 1] < b[4] + 6) &
                       (pts[:, 2] > b[2] - 6) & (pts[:, 2] < b[5] + 6)]
            if not len(near):
                continue
            inside = ST.inside(pm, near)
            if inside.any():
                # PREFIX: a stock horn is "horn-pan-shorn-hub" and
                # "horn-pan-shorn-arm", so an exact-name exemption never
                # fired and every servo reported as buried in its own horn.
                if any(pre == a_ and pn.startswith(b_) for a_, b_ in BY_DESIGN):
                    continue          # the drive train, not a clash
                buried += int(inside.sum())
                worst, who = -1.0, pn
                continue
            pv = np.asarray(pm.V)
            step = max(1, len(pv) // 4000)
            pv = pv[::step]
            d = np.sqrt(((near[:, None, :] - pv[None, :, :]) ** 2).sum(-1)).min()
            if d < worst:
                worst, who = float(d), pn
        if buried:
            fails.append(f"{label}: {buried} vertices inside {who}")
            print(f"  FAIL {label:30s} {buried} pts INSIDE {who}")
        else:
            flag = "tight" if worst < 0.8 else ""
            if worst < 0.8:
                warns.append(f"{label}: only {worst:.2f} mm from {who}")
            print(f"  ok   {label:30s} {worst:5.2f} mm to {who}  {flag}")

    # ---- 3. enclosure ----
    # Straight up from each board's top face. If the ray exits the model,
    # that board is open to the room.
    print("\nenclosure: is anything directly above each board?")
    for label, pre in (("ESP32-S3", "esp32-"), ("amplifier", "amp-"),
                       ("microphone", "mic-"), ("speaker", "spk-"),
                       ("LED ring", "ring-")):
        parts = [(n, m) for n, m in M.items() if n.startswith(pre)]
        b0 = [min(m.bounds()[i] for _n, m in parts) for i in range(3)]
        b1 = [max(m.bounds()[i + 3] for _n, m in parts) for i in range(3)]
        grid = []
        for gx in np.linspace(b0[0] + 1, b1[0] - 1, 7):
            for gy in np.linspace(b0[1] + 1, b1[1] - 1, 7):
                grid.append((gx, gy))
        covered = 0
        for gx, gy in grid:
            col = np.array([[gx, gy, z] for z in
                            np.arange(b1[2] + 1.0, b1[2] + 130.0, 1.5)])
            hit = np.zeros(len(col), bool)
            for _pn, pm in printed:
                pb = pm.bounds()
                if not (pb[0] < gx < pb[3] and pb[1] < gy < pb[4]):
                    continue
                hit |= ST.inside(pm, col)
                if hit.any():
                    break
            covered += int(hit.any())
        pct = 100.0 * covered / len(grid)
        ok = pct > 85.0
        # the LED ring is SUPPOSED to be open upward -- it shines out
        if pre == "ring-":
            print(f"  --   {label:16s} {pct:5.1f}% covered  "
                  f"(open by design: this is the light)")
            continue
        if not ok:
            fails.append(f"{label}: only {pct:.0f}% of it is covered")
        print(f"  {'ok  ' if ok else 'FAIL'} {label:16s} {pct:5.1f}% covered")

    # ---- what this cannot tell you ----
    print("\nCovered elsewhere, no longer asserted:")
    print("  - wire routing -> v2_wire_check searches free space for a real")
    print("    cable path per component; it does not take routing on trust.")
    print("  - thread strength, torque, thermal -> v2_margins computes each")
    print("    from this geometry against constants it names out loud.")
    print("\nStill genuinely unknown until one is printed:")
    print("  - layer adhesion on YOUR printer and filament, which is what")
    print("    the printed thread's 25 MPa assumption really rests on.")

    print("=" * 68)
    if fails:
        print(f"{len(fails)} PROBLEM(S):")
        for f in fails:
            print("  - " + f)
        return 1
    if warns:
        print(f"no blockers; {len(warns)} tight fit(s) to watch:")
        for w in warns:
            print("  - " + w)
        return 0
    print("every component present, clear, and enclosed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

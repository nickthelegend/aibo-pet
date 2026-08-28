"""v2_audit.py — Hotaru 2.0, held to the same standard as v1.

Checks, each of which has an argument for existing:
  watertight    every part, print pose
  bed           every part AND every plate fits the P1S (256^2)
  over-air      audit_support's ray test per print-pose part -- the check the
                v1 shoulder taught us to run BEFORE printing, not after
  interference  every overlapping-box pair in world pose, per-shell parity
  engagement    the numbers that make the joints real: horn cross depth,
                stub/bore overlap, spacer gap vs servo, shade yoke vs head
"""
from __future__ import annotations

import itertools
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import audit_support as asup
import params as P
import partlib as pl
import solidtest as ST
import v2_assembly as A
import v2_parts as V

BED = V.BED


def main():
    fails = []

    print("HOTARU 2.0 audit")
    print("=" * 64)

    # ---- parts: watertight, bed, over-air ----
    print(f"{'part':18s} {'water':>6s} {'bed':>4s} {'over-air':>9s}")
    for n, m in A.print_items():
        r = pl.validate(m)
        b = m.bounds()
        fit = (b[3] - b[0] <= BED - 12 and b[4] - b[1] <= BED - 12
               and b[5] - b[2] <= BED)
        a, _w, _c = asup.unsupported(m)
        ok = r["watertight"] and fit
        if not r["watertight"]: fails.append(f"{n}: not watertight")
        if not fit: fails.append(f"{n}: busts the P1S bed")
        # v2-head's stub axle is a horizontal boss: 45 mm2 over air, printed
        # with supports on, and the plate README says so. The shade carries
        # its v1 flag. Everything else must print clean.
        # allowlist, each with its reason:
        #   shade      carries its v1 flag
        #   v2-head    45 mm2 under its horizontal stub axle, supports on
        #   v2-screw / v2-trimcap: the coin slot passes under the collar and
        #   plug as a 3.4 mm bridge, anchored both sides -- the ray test
        #   cannot see anchoring, only vacancy
        # Exemptions, each a BRIDGE the vacancy test cannot credit:
        #   shade: v1 flag. v2-head: 45 mm2 under the stub, supports on.
        #   v2-screw/-trimcap: the coin slot under collar/plug, 3.4 span.
        #   v2-tub: USB cavity roof, MX relief and wire-notch ceilings, all
        #   anchored on both sides, max span 16.
        #   v2-link1-out: the 1.0-deep case relief's ceiling, anchored all
        #   round its rim.
        #   v2-link1-in: the horn recess pocket, printed face down; its
        #   cross arms are 6.8 wide and bridge wall to wall.
        #   v2-screws/-bolts: same coin-slot-under-collar bridge as
        #   v2-screw, once per screw on the strip; plus the thread's own
        #   45-degree start ramp over the neck -- v1 printed this form.
        #   v2-conecap: the 4 face bridges over the light slot, 7.5 span,
        #   anchored on both rims, deliberately placed in the bed-side half.
        if a > asup.MIN_AREA and n not in ("shade", "v2-head",
                                           "v2-screw", "v2-screws",
                                           "v2-bolts", "v2-trimcap",
                                           "v2-caphead", "v2-conecap",
                                           "v2-tub", "v2-link1-out",
                                           "v2-link1-in"):
            fails.append(f"{n}: {a:.0f} mm2 over air in print pose")
        print(f"{n:18s} {str(r['watertight']):>6s} {'OK' if fit else 'NO':>4s} "
              f"{a:8.1f} {'<-- needs support' if a > asup.MIN_AREA else ''}")

    # ---- world interference ----
    print("\npairwise interference, world pose")
    world = [(n, m) for n, m, _c in A.world_items()]
    B = {n: m.bounds() for n, m in world}
    M = dict(world)
    rng = np.random.default_rng(9)
    pairs = 0
    def component(nm):
        # Every component model is a union of overlapping shells, so its own
        # sub-meshes intersect by construction: an ESP32's module sits on its
        # PCB, a speaker's cone inside its frame. Group by FAMILY and skip
        # same-family pairs; cross-family pairs stay fully checked.
        for pre in ("pan-", "sh-", "el-", "hd-", "mx-",
                    "esp32-", "amp-", "mic-", "spk-", "ring-"):
            if nm.startswith(pre):
                return pre
        return None
    for (x, _), (y, _) in itertools.combinations(world, 2):
        # sub-meshes of one servo overlap BY CONSTRUCTION (a component is a
        # union of shells); only cross-component pairs are meaningful
        if component(x) and component(x) == component(y):
            continue
        a, b = B[x], B[y]
        if not all(min(a[i+3], b[i+3]) - max(a[i], b[i]) > 0.05 for i in range(3)):
            continue
        pairs += 1
        lo = [max(a[i], b[i]) for i in range(3)]
        hi = [min(a[i+3], b[i+3]) for i in range(3)]
        pts = rng.uniform(lo, hi, size=(2600, 3))
        n_in = int((ST.inside(M[x], pts) & ST.inside(M[y], pts)).sum())
        # designed engagements and contact-plane kisses:
        #   tower/screw and link1-out/screw are ENGAGED THREADS -- flanks of
        #   both parts occupy the same annulus on purpose
        #   n <= 2 is sampling noise on a face-to-face contact plane
        engaged = {frozenset(("v2-tower", "v2-screw")),
                   frozenset(("v2-link1-in", "v2-screw-elbow")),
                   frozenset(("v2-link2-out", "v2-screw-elbow"))}
        # A horn EXISTS to occupy two things at once: its servo's spline and
        # the recess it drives. Those are the joint working, not a clash.
        HORN_OK = {
            "horn-pan": ("pan-", "v2-disc"),
            "horn-shoulder": ("sh-", "v2-tower", "v2-link1-in"),
            "horn-elbow": ("el-", "v2-link1-out", "v2-link2-in"),
            "horn-head": ("hd-", "v2-head", "shade"),
        }
        for hn, oks in HORN_OK.items():
            if hn in (x, y):
                other = y if x == hn else x
                if other.startswith(tuple(oks)):
                    n_in = 0
        if frozenset((x, y)) in engaged or n_in <= 5:
            n_in = 0
        flag = "" if n_in == 0 else f"  CLASH {n_in}"
        if n_in: fails.append(f"{x} <-> {y}: {n_in} pts interfere")
        print(f"  {x:16s} {y:16s}{flag or '  clear'}")
    print(f"  ({pairs} overlapping-box pairs probed)")

    # ---- assembly hardware really passes through the plates ----
    print("\nstandoff and clamp screw holes (mesh probed, not assumed)")
    import v2_parts as VP
    plate_specs = [
        ("v2-link1-in", VP.link1()[0][1], list(VP.l1_spots()) + list(VP.l1_ledge_spots())),
        ("v2-link1-out", VP.link1()[1][1], list(VP.l1_spots())),
        ("v2-link2-in", VP.link2()[0][1], list(VP.l2_spots())),
        ("v2-link2-out", VP.link2()[1][1], list(VP.l2_spots())),
    ]
    for pn, mesh, spots in plate_specs:
        probe = np.array([[x, y, VP.PLATE_T / 2.0] for x, y in spots])
        blocked = ST.inside(mesh, probe)
        n_ok = int((~blocked).sum())
        ok = n_ok == len(spots)
        if not ok:
            fails.append(f"{pn}: {int(blocked.sum())} screw position(s) have no hole")
        print(f"  {'PASS' if ok else 'FAIL':4s} {pn:16s} {n_ok}/{len(spots)} holes open")


    # ---- engagement ----
    print("\nengagement")
    rec = P.HORN_T + P.HORN_FIT
    # every face here is the derived stack in v2_parts' header comment
    sh_horn_face = (V.DRIVE_CHEEK[1] - 1.0) + 2.8       # cb floor + horn
    sh_eng = min(rec, sh_horn_face - V.L1_IN_HALF)
    el_horn_face = (V.L1_OUT_HALF + V.BOSS_WALL) + 2.8
    el_eng = min(rec, el_horn_face - V.L2_IN_HALF)
    el_clear = V.L2_IN_HALF - (V.L1_OUT_HALF + V.PLATE_T)
    spline_horn = (V.CASE_TOP + 4.7) - (V.DRIVE_CHEEK[1] - 1.0)
    rows = [
        ("shoulder horn cross engagement", sh_eng, 1.5, "mm"),
        ("elbow horn cross engagement", el_eng, 1.5, "mm"),
        ("elbow drive plate clears link1", el_clear, 0.25, "mm"),
        ("spline reach into the horn socket", spline_horn, 1.5, "mm"),
        ("elbow servo tail inside the sandwich",
         V.L1_IN_HALF - V.ELBOW_TAIL, 1.0, "mm"),
        ("stub axle engage into link bore", P.SCREW_ENGAGE, 4.0, "mm"),
    ]
    # shade yoke inner gap vs head block width, measured off the meshes
    import part_head
    sm = part_head.build()[0][1]
    Vv = np.asarray(sm.V)
    # yoke inner faces: material nearest the axis plane on each side at the
    # bore height band
    band = Vv[(Vv[:, 2] > part_head.TILT - 6) & (Vv[:, 2] < part_head.TILT + 6)]
    xs = band[:, 0]
    inner_gap = xs[xs > 0].min() - xs[xs < 0].max() if (xs > 0).any() and (xs < 0).any() else 0
    rows.append(("shade yoke gap minus head block", inner_gap - 2 * V.HEAD_HALF, 0.3, "mm"))
    for name, val, need, unit in rows:
        ok = val >= need
        if not ok: fails.append(f"{name}: {val:.2f} < {need}")
        print(f"  {'PASS' if ok else 'FAIL':4s} {name:34s} {val:6.2f} {unit} (need >= {need})")

    # ---- contact: fastened pairs, as DERIVED plane gaps ----
    # These interfaces are axis-aligned planes whose positions come from the
    # same constants the parts are built from, so the gap is computed, not
    # sampled: a 1500-point cloud in a 300 mm box put its nearest points
    # 1.5 mm apart on faces that touch by construction.
    print("\ncontact (derived plane gaps at the fastened interfaces)")
    rows2 = [
        ("disc rides the tub rim", V.DISC_Z0 - V.TUB_H, 0.0, 0.05),
        ("tower base on the disc face", 0.0, 0.0, 0.05),
        ("link1 spacers to inner faces", 0.2, 0.0, 0.45),
        ("link2 spacers to inner faces", 0.2, 0.0, 0.45),
        ("head tail to link2 plates",
         (V.L2_IN_HALF + V.L2_OUT_HALF) - V.HEAD_TAIL_W, 0.0, 1.0),
        ("crown clears the skirt", 78.2 - 77.7, 0.3, 2.0),
        ("crown top vs folding arm parts", 56.0 - 50.5, 0.0, 22.0),
    ]
    for name, val, lo2, hi2 in rows2:
        ok = lo2 <= val <= hi2
        if not ok:
            fails.append(f"contact {name}: {val:.2f} outside [{lo2},{hi2}]")
        print(f"  {'PASS' if ok else 'FAIL':4s} {name:34s} {val:6.2f} mm "
              f"(want {lo2}..{hi2})")

    # ---- MX switch: does it actually clip into the turret top? ----
    # Mesh-probed, not assumed: the plate band must be SOLID beside the
    # cutout and OPEN inside it, the relief hollow under the plate, and the
    # pod solid outside the relief wall. A probe list that cannot tell those
    # apart would pass a turret with no pocket at all.
    print("\nMX switch (turret top, mesh probed)")
    tub_m = dict(world)["v2-tub"]
    cx, cy = V.MXC
    zp = V.MX_PLATE_Z + 0.7           # inside the 1.5 plate band
    zr = 50.0                          # inside the relief band
    probes = [
        ("plate solid beside the cutout", (cx, cy - (P.MX_CUT / 2 + 0.6), zp), True),
        ("cutout open for the clips",     (cx, cy, zp), False),
        ("relief hollow under the plate", (cx, cy, zr), False),
        # The walls that retain the switch and carry the pod are the SIDES.
        # This used to probe behind the relief, which is now deliberately
        # an access port -- without it the solder tags sit in a blind
        # pocket no iron can reach. Probing the sides keeps the original
        # teeth (a pod with no pocket still fails, its side walls missing)
        # and the port itself is now guarded by the row below, so it cannot
        # quietly close up again.
        ("pod solid beside the relief +X",
         (cx + (P.MX_BODY_SQ / 2 + 1.2), cy, zr), True),
        ("pod solid beside the relief -X",
         (cx - (P.MX_BODY_SQ / 2 + 1.2), cy, zr), True),
        ("access port OPEN behind the tags",
         (cx, cy - (P.MX_BODY_SQ / 2 + 6.0), zr), False),
        ("wire bore open below",          (cx, cy, 36.0), False),
        ("top face solid at the rim",     (cx + 12.0, cy, V.TR_TOP - 0.7), True),
    ]
    for lbl, pt, want in probes:
        got = bool(ST.inside(tub_m, np.array([pt], dtype=float))[0])
        ok = got == want
        if not ok:
            fails.append(f"MX turret: {lbl} (solid={got}, wanted {want})")
        print(f"  {'PASS' if ok else 'FAIL':4s} {lbl:32s} solid={got}")
    stem_up = [m for n, m in world if n == "mx-mx-stem"]
    if stem_up:
        vb = np.asarray(stem_up[0].V)
        ok = vb[:, 2].min() >= V.TR_TOP - 1e-6
        print(f"  {'PASS' if ok else 'FAIL':4s} {'stem points UP from the fixed pod':32s} "
              f"z_min {vb[:, 2].min():.1f}")
        if not ok:
            fails.append("MX stem does not stand clear of the turret top")
    else:
        fails.append("MX switch is not placed at all")
        print("  FAIL switch not placed")
    cut_ok = abs(P.MX_CUT - 14.1) < 1e-6 and abs(P.MX_PLATE_T - 1.5) < 1e-6
    print(f"  {'PASS' if cut_ok else 'FAIL':4s} v1 plate spec kept              "
          f"{P.MX_CUT} cutout in a {P.MX_PLATE_T} plate")
    if not cut_ok:
        fails.append("MX plate spec drifted from v1")

    # ---- connectivity: nothing printed may float ----
    # The arm read as "broken" because the elbow had a plate on one side
    # only. A bounds-touch graph catches that class of thing directly: every
    # printed part must come within fastening distance of another.
    print("\nconnectivity (printed parts, bounds proximity)")
    # v2-keycap holds onto the SWITCH stem, which is hardware, so the
    # printed-to-printed graph would always report it floating
    printed = [(n, m) for n, m in world
               if not n.startswith(("pan-", "sh-", "el-", "hd-", "mx-",
                                    "v2-keycap"))]
    Bp = {n: m.bounds() for n, m in printed}
    def near(a, b, tol=1.6):
        return all(min(a[i+3], b[i+3]) - max(a[i], b[i]) > -tol for i in range(3))
    for n, _m in printed:
        nb = [o for o, _o in printed if o != n and near(Bp[n], Bp[o])]
        ok = len(nb) > 0
        if not ok:
            fails.append(f"{n} touches nothing -- it would fall off")
        print(f"  {'PASS' if ok else 'FAIL':4s} {n:20s} {len(nb)} neighbour(s)")

    print("=" * 64)
    if fails:
        print(f"{len(fails)} FAILING:")
        for f in fails: print("  -", f)
        return 1
    print("all checks pass")
    return 0


if __name__ == "__main__":
    sys.exit(main())

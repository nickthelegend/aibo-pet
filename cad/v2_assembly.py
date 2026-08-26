"""v2_assembly.py — Hotaru 2.0 world pose, print pose, and plates.

The joint stack, +X is the drive side everywhere:

  pan       MG996R in the tub, spline up on (0,0), horn into the disc.
  shoulder  tower on the disc; horn outside the +X cheek at x 18.45..21.25;
            link1-in recess face at 18.85 -> the cross engages 2.4.
  elbow     MG996R inside link1's far end; spline through link1-in's boss
            bore; horn at 22.85..25.65; link2-in inner face at 26.85 -> the
            cross engages 1.9 of its 3.1 recess. Thinner than v1's full
            depth; carried as a measured number in the audit, not a hope.
  idlers    every -X side rides a printed threaded stub + yoke-screw, v1's
            proven pair, unchanged.
  head      MG996R in the head block between link2's plates; the cone shade
            drops onto the same interface it always had.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

import components as CO
import params as P
import part_head
import part_horn
import partlib as pl
import v2_parts as V

SH_AX_Z = V.DISC_Z0 + V.DISC_T + V.TWR_AXIS_Z      # 95.5? computed below
EL_AX_Z = None                                      # filled in world_items


def world_items():
    out = []
    for n, m, c in V.tub() + V.disc() + V.tower():
        out.append((n, m.copy(), c))

    z_sh = V.DISC_Z0 + V.DISC_T + V.TWR_AXIS_Z
    z_el = z_sh + V.L1
    z_hd = z_el + V.L2

    # ---- link1. Plates flat in XY; rotate_y(-90) maps length->+Z and
    # thickness->-X, so a plate's local z0 face lands at the translate dx.
    # Where the z0 face (the one carrying the recess or the boss web) must
    # face -X we add rotate_z(180), which mirrors x and y.
    for n, m, c in V.link1():
        q = m.copy()
        q.rotate_y(-90.0)
        if n == "v2-link1-in":
            q.rotate_z(180.0)
            q.translate(dx=V.L1_IN_HALF, dz=z_sh)      # plate 25.75..29.75
        elif n == "v2-link1-out":
            q.translate(dx=-V.L1_OUT_HALF, dz=z_sh)    # plate -30.15..-26.15
        elif n == "v2-link1-spacers":
            q.translate(dx=V.L1_IN_HALF - 0.2, dz=z_sh)
        elif n == "v2-link1-ledges":
            q.translate(dx=V.L1_IN_HALF - 0.05, dz=z_sh)
        out.append((n, q, c))

    # ---- link2: drive -X (recess toward +X onto the elbow horn), idler +X
    for n, m, c in V.link2():
        q = m.copy()
        q.rotate_y(-90.0)
        if n == "v2-link2-in":
            q.translate(dx=-V.L2_IN_HALF, dz=z_el)     # plate -34.45..-30.45
        elif n == "v2-link2-out":
            q.rotate_z(180.0)
            q.translate(dx=V.L2_OUT_HALF + V.PLATE_T, dz=z_el)  # 30.05..34.05
        elif n == "v2-link2-spacers":
            q.translate(dx=V.L2_OUT_HALF - 0.2, dz=z_el)
        out.append((n, q, c))

    # ---- head block between link2's plates at the far end; the tail is
    # asymmetric with the plates, so it shifts by half the difference
    xc = (V.L2_OUT_HALF - V.L2_IN_HALF) / 2.0
    for n, m, c in V.head_block():
        q = m.copy()
        q.rotate_x(90.0)
        q.translate(dx=xc, dy=15.0, dz=z_hd - 22.0)
        out.append((n, q, c))

    # ---- the v1 cone shade, unchanged, on the head axis
    sname, smesh, scol = part_head.build()[0]
    sm = smesh.copy()
    sm.translate(dz=-part_head.TILT)
    sm.rotate_x(180.0)
    sm.translate(dx=xc, dz=z_hd)
    out.append((sname, sm, scol))

    # ---- the hardware that visually and physically closes the joints ----
    scr = V.v2_screw()[0][1].copy()
    scr.rotate_y(90.0)
    scr.translate(dx=-(V.L1_OUT_HALF + V.PLATE_T + 3.2), dz=z_sh)
    out.append(("v2-screw", scr, V.COLORS["v2-accent"]))

    scr2 = V.v2_screw()[0][1].copy()
    scr2.rotate_y(-90.0)
    scr2.translate(dx=V.L2_OUT_HALF + V.PLATE_T + 3.2, dz=z_el)
    out.append(("v2-screw-elbow", scr2, V.COLORS["v2-accent"]))

    # No trim cap at the elbow. Its job was to plug the hub bore -- but the
    # HORN occupies that bore, which is the whole point of the joint, and
    # the cap was driving 4 mm into it. A cosmetic part that fights the
    # drive train is not worth a redesign, and deleting it takes a piece off
    # the plate rather than adding one.

    # ---- the real printed horns: what actually carries torque from each
    # spline into its plate. v1's part_horn, unchanged.
    HORNS = {n: m for n, m, _c in part_horn.build()}
    HC = V.COLORS["v2-accent"]
    hp = HORNS["horn-mg996r"].copy()
    hp.rotate_x(180.0)
    hp.translate(dz=V.DISC_Z0 + (P.HORN_T + P.HORN_FIT))
    out.append(("horn-pan", hp, HC))
    hs = HORNS["horn-mg996r"].copy()
    hs.rotate_y(90.0)
    hs.translate(dx=V.DRIVE_CHEEK[1] - 1.0, dz=z_sh)
    out.append(("horn-shoulder", hs, HC))
    he = HORNS["horn-mg996r"].copy()
    he.rotate_y(-90.0)
    he.translate(dx=-(V.L1_OUT_HALF + V.BOSS_WALL), dz=z_el)
    out.append(("horn-elbow", he, HC))
    hh = HORNS["horn-mg996r"].copy()
    hh.rotate_y(90.0)
    # cross face at the COUNTERBORE floor, exactly like the shoulder cheek:
    # 1.0 into the nose face buys the same 1.7 of spline in the socket
    hh.translate(dx=xc + V.HEAD_HALF - V.HEAD_CBORE_T, dz=z_hd)
    out.append(("horn-head", hh, HC))

    # ---- the servos: the joints read as connected because the thing that
    # connects them is in the picture ----
    def servo(kind):
        return [(n2, mm.copy(), c2)
                for n2, mm, c2 in (CO.mg996r() if kind == "mg" else CO.sg90())]

    def spline_xy(ms):
        V2 = np.vstack([np.asarray(mm.V) for _n, mm, _c in ms])
        top = V2[V2[:, 2] > V2[:, 2].max() - 2.0]
        return float(top[:, 0].mean()), float(top[:, 1].mean()), float(V2[:, 2].max())

    ms = servo("mg"); sx, sy, t = spline_xy(ms)
    for n2, mm, c2 in ms:
        mm.translate(dx=-sx, dy=-sy, dz=V.FLOOR)
        out.append((f"pan-{n2}", mm, c2))
    ms = servo("mg"); sx, sy, t = spline_xy(ms)
    for n2, mm, c2 in ms:
        mm.translate(dx=-sx, dy=-sy, dz=-t)
        mm.rotate_y(90.0)
        mm.translate(dx=V.CASE_TOP + 4.7, dz=z_sh)
        out.append((f"sh-{n2}", mm, c2))
    ms = servo("mg"); sx, sy, t = spline_xy(ms)
    for n2, mm, c2 in ms:
        mm.translate(dx=-sx, dy=-sy, dz=-t)
        mm.rotate_z(180.0)
        mm.rotate_y(-90.0)
        mm.translate(dx=-(V.L1_OUT_HALF + 4.7), dz=z_el)
        out.append((f"el-{n2}", mm, c2))
    ms = servo("mg"); sx, sy, t = spline_xy(ms)
    for n2, mm, c2 in ms:
        mm.translate(dx=-sx, dy=-sy, dz=-t)
        mm.rotate_y(90.0)
        # spline tip at the DERIVED station the pockets were cut for --
        # HEAD_HALF is the drive face, 0.7 short of the tip, and using it
        # here sank the whole servo 0.7 into the pocket floor
        mm.translate(dx=xc + V.HEAD_SPLINE_TIP, dz=z_hd)
        out.append((f"hd-{n2}", mm, c2))

    # ---- the real electronics, on the stations the tub builds ----
    for parts, xy, z, rz in (
            (CO.esp32_s3(),   V.ESP_XY, V.FLOOR + V.ESP_POST, V.ESP_ROT),
            (CO.max98357a(),  V.AMP_XY, V.FLOOR + V.AMP_POST, 0.0),
            (CO.inmp441(),    V.MIC_XY, V.MIC_Z, 90.0)):
        for n2, mm, c2 in parts:
            q = mm.copy()
            if rz:
                q.rotate_z(rz)
            q.translate(dx=xy[0], dy=xy[1], dz=z)
            out.append((n2, q, c2))
    # Speaker on edge against the +X wall, cone facing out. rotate_y(90)
    # then rotate_x(90) maps local (x,y,z) -> world (z, x, y): the 40 mm
    # length lands along Y between the rails, the 20 mm width stands up in
    # Z, and the cone axis points +X at the grille. The first attempt used
    # rotate_z after rotate_y and put the frame 8 mm below the tub floor.
    for n2, mm, c2 in CO.speaker2040():
        q = mm.copy()
        q.rotate_y(90.0)
        q.rotate_x(90.0)
        q.translate(dx=V.SPK_XY[0], dy=V.SPK_XY[1], dz=V.SPK_Z)
        out.append((n2, q, c2))

    # ---- the MX switch, in the turret top, stem UP ----
    # The component's own origin is the clip-plate top, which is exactly
    # V.TR_TOP: no rotation at all, the switch drops straight in. The body
    # hangs into the pod's relief, the pins into the wire bore.
    for n2, mm, c2 in CO.mx_switch():
        q = mm.copy()
        q.translate(dx=V.MXC[0], dy=V.MXC[1], dz=V.TR_TOP)
        out.append((f"mx-{n2}", q, c2))

    # printed keycap over the stem: stem pocket ceiling rests on the stem
    # top; the skirt's inner relief wraps the upper housing without touching
    kc = V.v2_keycap()[0][1].copy()
    kc.rotate_x(180.0)
    kc.translate(dx=V.MXC[0], dy=V.MXC[1],
                 dz=V.TR_TOP + P.MX_UPPER_H + P.MX_STEM_UP + 1.5)
    out.append(("v2-keycap", kc, V.COLORS["v2-accent"]))

    # the elbow's blue cap, back on: the joint reads symmetrical again
    tc = V.v2_trimcap()[0][1].copy()
    tc.rotate_y(90.0)
    # cup mouth toward the plate: the pocket swallows the proud hub end
    tc.translate(dx=-(V.L2_IN_HALF + V.PLATE_T) - 3.2, dz=z_el)
    out.append(("v2-trimcap", tc, V.COLORS["v2-accent"]))

    # the head hub's cap: RETENTION, not just trim -- glued to the hub, its
    # flange overlaps the drive plate's drop-in slot rails, closing the path
    # the shade came in by. Its own keyhole slot points up: it drops down
    # the corridor between the drive plate and link2-out.
    tc2 = V.v2_caphead()[0][1].copy()
    tc2.rotate_y(-90.0)
    tc2.translate(dx=xc + V.HEAD_HALF + P.HORN_HUB_T - V.HEAD_CBORE_T + 1.2,
                  dz=z_hd)
    out.append(("v2-caphead", tc2, V.COLORS["v2-accent"]))
    return out


# ------------------------------------------------------------- printing ----
def print_items():
    """Everything in print pose, flat side down, bottoms on Z=0."""
    out = []
    for n, m, c in (V.tub() + V.disc() + V.tower() + V.link1() + V.link2()
                    + V.head_block() + V.clamp_bars()
                    + V.v2_screw() + V.v2_trimcap() + V.v2_caphead()
                    + V.v2_keycap() + V.v2_horns()):
        q = m.copy()
        if n == "v2-disc":
            q.rotate_x(180.0)          # skirt up -> face down, prints flat
        # "-in" plates flip so the horn recess prints UP -- EXCEPT link1-in,
        # whose outer face now grows the elbow's stub axle. Recess and stub
        # are on opposite faces and only one can face up. The stub wins: it
        # is a 5.6 vertical boss that prints perfectly standing, while the
        # recess face-down is a 3.1 pocket bridging between anchored walls.
        # Flipping it instead stood the whole plate on the stub: 4939 mm2.
        if n.endswith("-in") and n != "v2-link1-in":
            q.rotate_x(180.0)
        b = q.bounds()
        q.translate(dz=-b[2])
        out.append((n, q))
    # the shade prints exactly as v1 ships it
    sn, sm, _sc = part_head.build()[0]
    q = sm.copy(); b = q.bounds(); q.translate(dz=-b[2])
    out.append(("shade", q))
    return out


PLATES = [
    ("v2-plate-1-core", "tub + disc + tower: the whole base in one go",
     ["v2-tub", "v2-disc", "v2-tower"]),
    ("v2-plate-2-arm", "every link plate, spacers, head block, clamps",
     ["v2-link1-in", "v2-link1-out", "v2-link1-spacers",
      "v2-link2-in", "v2-link2-out", "v2-link2-spacers",
      "v2-head", "v2-clamps"]),
    ("v2-plate-3-shade", "the cone, unchanged from v1",
     ["shade"]),
]

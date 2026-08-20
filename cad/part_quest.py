"""part_quest.py — a Quest 3S headset and its two Touch Plus controllers.

We capture teleoperation on a Quest 3S today, so the rig belongs on the site
next to the Band. This is OUR likeness of that hardware for illustration: no
Meta marks, no logos, dimensions approximate to within a few millimetres.
It is a picture of equipment we use, not a replica and not a Meta product.

The controllers are RINGLESS. Quest 3 and 3S both ship Touch Plus, which
dropped the tracking ring every Oculus controller had carried since the Rift
and moved to IR LEDs in the body plus hand-tracking fusion. Modelling them
with rings would be modelling the Quest 2 and would read as wrong to anybody
who owns a headset.

Same rules as every other part here: shapely 2D booleans lofted into
watertight shells, no 3D CSG, one object is a union of overlapping shells.

Frame: +X right, +Y forward (away from the face), +Z up.
"""
from __future__ import annotations

import math
import os
import sys

from shapely import affinity
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import partlib as pl

OVL = pl.OVL
SEG = 64

# ----------------------------------------------------------- parameters ----
VIS_W, VIS_H, VIS_D = 162.0, 98.0, 90.0     # visor envelope
FACE_W, FACE_H = 124.0, 92.0                # where it meets the face
STRAP_W, STRAP_T = 26.0, 9.0

CTRL_HEAD_W, CTRL_HEAD_L = 46.0, 44.0
CTRL_GRIP_L = 96.0
CTRL_TILT = 24.0                            # grip rake off vertical

COLOURS = {
    "quest-shell":   "#E9E9EE",
    "quest-front":   "#26262B",
    "quest-pod":     "#1A1A1F",
    "quest-lens":    "#0E0E12",
    "quest-face":    "#1E1E23",
    "quest-strap":   "#33333A",
    "ctrl-l-shell":  "#E9E9EE",
    "ctrl-l-dark":   "#26262B",
    "ctrl-r-shell":  "#E9E9EE",
    "ctrl-r-dark":   "#26262B",
}


def loft_chain(profs, zs):
    """One watertight shell through a stack of profiles.

    Every profile must have the same vertex count, which rounded_rect() and
    circle() both guarantee for a fixed seg, so the walls index-align without
    resample_ring().
    """
    m = pl.Mesh(weld=True)
    rings = [pl._rings(p)[0] for p in profs]
    for i in range(len(zs) - 1):
        m.add_loft_wall(rings[i], zs[i], rings[i + 1], zs[i + 1])
    m.add_cap(profs[0], zs[0], up=False)
    m.add_cap(profs[-1], zs[-1], up=True)
    return m


def rr(w, h, r):
    return pl.rounded_rect(w, h, r, seg=14)


# -------------------------------------------------------------- headset ----
def headset():
    """Visor shell, dark front fascia with the camera cluster, facial
    interface and the strap arms. Built along local +Z then stood up so +Y
    is forward."""
    parts = {}

    # visor: swells slightly just behind the front face, then tapers to the
    # facial interface
    zs =    [0.0,             12.0,            52.0,            74.0,        90.0]
    profs = [rr(VIS_W, VIS_H, 26),
             rr(VIS_W + 5, VIS_H + 3, 28),
             rr(VIS_W - 3, VIS_H, 30),
             rr(FACE_W + 16, FACE_H + 2, 32),
             rr(FACE_W, FACE_H, 34)]
    shell = loft_chain(profs, zs)

    # strap arms, one per side, sweeping back from the visor mid-line
    for sx in (-1, 1):
        shell += pl.prism(
            affinity.translate(rr(STRAP_T, STRAP_W, 4.0),
                               sx * (VIS_W / 2 - 2), 0.0),
            74.0, 128.0)
    parts["quest-shell"] = shell

    # dark front fascia, standing slightly proud of the shell
    front = pl.Mesh()
    front += pl.prism(rr(VIS_W - 18, VIS_H - 18, 22), -2.5, 6.0)
    parts["quest-front"] = front

    # camera cluster. Two outboard pods and a centre pair, which is the
    # arrangement a 3S actually reads as from the front.
    lens = pl.Mesh()
    pod = pl.Mesh()
    for sx in (-1, 1):
        # raised pod carrying the stacked camera pair on each side
        pod += loft_chain(
            [affinity.translate(pl.circle(40.0, SEG), sx * 52, 2),
             affinity.translate(pl.circle(37.0, SEG), sx * 52, 2)],
            [-7.0, -1.0])
    parts["quest-pod"] = pod
    for x, y in [(-52, 15), (-52, -12), (52, 15), (52, -12), (-9, 24), (9, 24)]:
        d = 14.0 if abs(x) > 40 else 8.5
        lens += pl.prism(affinity.translate(pl.circle(d, SEG), x, y), -9.5, -6.5)
    parts["quest-lens"] = lens

    # facial interface: a soft collar around the eye box
    face = pl.Mesh()
    face += loft_chain([rr(FACE_W, FACE_H, 34), rr(FACE_W - 14, FACE_H - 12, 34)],
                       [90.0 - OVL, 112.0])
    parts["quest-face"] = face

    # A band round the back of the head, not a shelf: narrow, and inset from
    # the arms so it reads as fabric spanning between them.
    strap = pl.Mesh()
    strap += pl.prism(rr(VIS_W - 26, STRAP_W - 8, 8.0), 122.0, 130.0)
    parts["quest-strap"] = strap

    # Stand it up. rotate_x(+90) maps local +Z (depth, front to face) onto
    # world -Y, which puts the camera face at +Y where a viewer expects it.
    # Using -90 here is what had every earlier render showing the back.
    for m in parts.values():
        m.rotate_x(90.0)
    return parts


# ----------------------------------------------------------- controller ----
def controller(side):
    """Touch Plus: rounded head carrying the stick and two face buttons, a
    raked grip, and a trigger. No tracking ring."""
    shell = pl.Mesh()
    dark = pl.Mesh()

    # head: rounded and domed, widest just under the top deck
    shell += loft_chain(
        [rr(CTRL_HEAD_W - 14, CTRL_HEAD_L - 14, 11),
         rr(CTRL_HEAD_W - 2, CTRL_HEAD_L - 3, 15),
         rr(CTRL_HEAD_W, CTRL_HEAD_L, 16),
         rr(CTRL_HEAD_W - 7, CTRL_HEAD_L - 8, 15)],
        [-8.0, 2.0, 14.0, 24.0])

    # grip: an oval section, not a circle, so it reads as something shaped to
    # a hand; rakes back under the rear of the head
    def oval(w, h):
        return affinity.scale(pl.circle(2.0, SEG), xfact=w / 2, yfact=h / 2)
    grip = loft_chain(
        [oval(34, 31), oval(33, 30), oval(31, 28), oval(27, 25)],
        [0.0, 32.0, 66.0, CTRL_GRIP_L])
    grip.rotate_x(180.0 - CTRL_TILT)
    grip.translate(dy=-7.0, dz=8.0)
    shell += grip

    # thumbstick and two face buttons, on the top deck
    sx = -1 if side == "l" else 1
    dark += pl.prism(affinity.translate(pl.circle(19.0, SEG), sx * -9.0, 8.0),
                     24.0, 28.5)
    shell += pl.prism(affinity.translate(pl.circle(15.0, SEG), sx * -9.0, 8.0),
                      27.5, 31.0)
    for by in (-6.0, -16.0):
        dark += pl.prism(affinity.translate(pl.circle(11.0, SEG), sx * 11.0, by),
                         24.0, 27.0)

    # trigger, on the forward face of the grip
    trig = pl.prism(pl.rounded_rect(20.0, 13.0, 5.0, seg=10), 0.0, 9.0)
    trig.rotate_x(-90.0 - CTRL_TILT)
    trig.translate(dy=-19.0, dz=-9.0)
    dark += trig

    return {f"ctrl-{side}-shell": shell, f"ctrl-{side}-dark": dark}


def build():
    out = {}
    out.update(headset())
    out.update(controller("l"))
    out.update(controller("r"))
    return out


def assembled():
    """Headset centred, controllers held out to either side and below, which
    is how the product shot everybody recognises is composed."""
    parts = build()
    for side, sx in (("l", -1), ("r", 1)):
        for key in (f"ctrl-{side}-shell", f"ctrl-{side}-dark"):
            m = parts[key]
            m.rotate_z(sx * 14.0)
            m.translate(dx=sx * 138.0, dy=-6.0, dz=-46.0)
    return parts


def main():
    ok = True
    for name, m in build().items():
        rep = pl.validate(m)
        b = m.bounds()
        size = tuple(round(b[i + 3] - b[i], 1) for i in range(3))
        if not rep["watertight"]:
            ok = False
        print(f"{name:14s} tris={len(m.F):6d} size={size} "
              f"{'OK' if rep['watertight'] else 'NOT WATERTIGHT ' + str(rep['problems'][:2])}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

"""joints.py — the shared joint/segment vocabulary.

EVERY arm segment is modelled with its long axis along +Z, yoke at the
bottom, servo cup at the top -- and that is also exactly how it prints.
Consequence: every cross-section is a print layer, so there is not one
bridge, overhang or support anywhere in the arm. The yoke's two plates
start as separate islands on the build plate and merge into the tube as
the print rises; the servo drops into the cup from the open top.

A joint is always driven on ONE side and bearing-supported on the other:
the drive plate captures the servo horn in a CROSS RECESS (the slot walls
carry the torque, the screws only retain it axially -- so clone horn screw
patterns stop mattering), and the idler plate rides a stub axle on the far
wall of the housing. Nothing hangs off the servo spline alone.

Local frame for a segment: Z=0 is the bottom of the yoke, X is the pivot
axis, Y is the swing direction.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import partlib as pl

OVL = pl.OVL

# housing half-sizes, derived -- the servo lies with H along X (shaft axis),
# L along Z (segment axis), W along Y (swing depth)
H_HX = P.MG_H / 2.0 + P.MG_FIT + P.JOINT_WALL          # 24.45
H_HY = P.MG_W / 2.0 + P.MG_FIT + P.JOINT_WALL          # 13.25
YK_X0 = H_HX + P.JOINT_GAP                             # yoke inner face
YK_X1 = YK_X0 + P.YOKE_PLATE_T                         # yoke outer face


def cyl_x(d, x0, x1, cy, cz, seg=24):
    """A cylinder whose axis runs along X, built as Z-slices (the kernel only
    extrudes in Z). Used for stub axles and their bores."""
    m = pl.Mesh()
    r = d / 2.0
    for i in range(seg):
        a0 = math.pi * i / seg
        a1 = math.pi * (i + 1) / seg
        z_hi = cz + r * math.cos(a0)
        z_lo = cz + r * math.cos(a1)
        w = r * math.sin(a1) if i < seg / 2 else r * math.sin(a0)
        w = max(w, r * math.sin(a0), r * math.sin(a1))
        if z_hi - z_lo < 1e-6:
            continue
        m += pl.prism(box(x0, cy - w, x1, cy + w), z_lo, z_hi)
    return m


def _disc_slices(d, cy, cz, seg=24):
    """[(z0, z1, half_width), ...] slicing a circle for X-axis bores."""
    out, r = [], d / 2.0
    for i in range(seg):
        a0, a1 = math.pi * i / seg, math.pi * (i + 1) / seg
        z_hi, z_lo = cz + r * math.cos(a0), cz + r * math.cos(a1)
        w = max(r * math.sin(a0), r * math.sin(a1))
        if z_hi - z_lo > 1e-6:
            out.append((z_lo, z_hi, w))
    return out


# ------------------------------------------------------------------ yoke ----
def yoke(axis_z, drive=True, idler=True):
    """The fork that straddles the previous joint's housing.

    drive plate (+X): cross recess for the servo horn + a through hub bore
                      so the horn's retaining screw stays reachable.
    idler plate (-X): bore riding the housing's stub axle, which the printed
                      yoke-screw then caps -- the screw head is SCREW_HEAD_D
                      against a bore of AXLE_D + AXLE_FIT, so the plate is
                      trapped between the housing wall and the head and the
                      segment cannot lift off.
    """
    z0, z1 = axis_z - P.YOKE_BELOW, axis_z + P.YOKE_ABOVE
    hy = P.YOKE_DEPTH / 2.0
    m = pl.Mesh()

    # ---- drive plate (+X) -------------------------------------------------
    plate = box(YK_X0, -hy, YK_X1, hy)
    rec_x = box(YK_X0 - OVL, -hy - 1, YK_X0 + P.HORN_T + P.HORN_FIT, hy + 1)
    arm_w = P.HORN_ARM_W + P.HORN_FIT
    hub = P.HORN_HUB_D + P.HORN_FIT
    openings = [
        # cross arm along Y (a Z band), and along Z (a Y band)
        (rec_x, axis_z - arm_w / 2, axis_z + arm_w / 2),
        (rec_x.intersection(box(-99, -arm_w / 2, 99, arm_w / 2)),
         axis_z - P.HORN_ARM_HALF, axis_z + P.HORN_ARM_HALF),
        # hub bores THROUGH the plate (square-ish: the cross carries torque)
        (box(YK_X0 - OVL, -hub / 2, YK_X1 + OVL, hub / 2),
         axis_z - hub / 2, axis_z + hub / 2),
    ]
    m += pl.banded(plate, z0, z1, openings) if drive else pl.prism(plate, z0, z1)

    # ---- idler plate (-X) -------------------------------------------------
    plate2 = box(-YK_X1, -hy, -YK_X0, hy)
    if idler:
        bore = [(box(-YK_X1 - OVL, -w, -YK_X0 + OVL, w), zl, zh)
                for zl, zh, w in _disc_slices(P.AXLE_D + P.AXLE_FIT, 0.0, axis_z)]
        m += pl.banded(plate2, z0, z1, bore)
    else:
        m += pl.prism(plate2, z0, z1)
    return m


# --------------------------------------------------------------- housing ----
def housing(axis_z, extra=()):
    """MG996R cup: open at the top so the servo drops straight in, spline
    exits +X, stub axle on -X. Tab slots trap the servo axially."""
    z_top = axis_z + P.MG_SHAFT_OFF + 8.0        # over the upper mounting tab
    z_bot = axis_z - (P.MG_L - P.MG_SHAFT_OFF) - P.JOINT_WALL
    outer = pl.rounded_rect(2 * H_HX, 2 * H_HY, 3.0)

    body_hx = P.MG_H / 2.0 + P.MG_FIT
    body_hy = P.MG_W / 2.0 + P.MG_FIT
    pocket = box(-body_hx, -body_hy, body_hx, body_hy)
    # The mounting tabs run along the SEGMENT axis (Z), so they need Z room,
    # not Y. The pocket already spans the body's full Z; the upper tab sits
    # in open air under the cap (which clamps it), the lower tab drops into
    # a slot that locates the servo axially -- so the tab HOLE pattern, the
    # one number in the brief that matches no MG996R drawing, never matters.
    tab_z0 = axis_z - (P.MG_L - P.MG_SHAFT_OFF) - P.MG_TAB_T - 0.4
    tab_z1 = axis_z - (P.MG_L - P.MG_SHAFT_OFF) + OVL
    tab_x = P.MG_H / 2.0 - P.MG_TAB_Z            # tab plane, offset from center
    tab_slot = box(tab_x - P.MG_TAB_T / 2 - 1.2, -body_hy - 3.0,
                   tab_x + P.MG_TAB_T / 2 + 1.2, body_hy + 3.0)

    spline = box(body_hx - OVL, -(P.MG_BOSS_D + 2.0) / 2, H_HX + 4.0,
                 (P.MG_BOSS_D + 2.0) / 2)
    openings = [
        (pocket, z_bot + P.JOINT_WALL, z_top),
        (tab_slot, tab_z0, tab_z1),
        (spline, axis_z - (P.MG_BOSS_D + 2.0) / 2, axis_z + (P.MG_BOSS_D + 2.0) / 2),
    ] + list(extra)
    m = pl.banded(outer, z_bot, z_top, openings)
    # Stub axle on the -X wall. The idler plate runs on its OD, and the
    # printed yoke-screw threads up its middle to stop the segment lifting
    # off -- see part_screw. Built along Z (the only axis the kernel threads
    # on) and laid over onto X, as its own watertight shell that the slicer
    # unions into the housing.
    axle = pl.threaded_bore(P.AXLE_D, P.SCREW_MAJOR, P.SCREW_PITCH,
                            0.0, P.SCREW_ENGAGE, clearance=P.SCREW_FIT / 2)
    axle.rotate_y(90.0)              # +Z -> +X
    axle.translate(dx=-H_HX - P.AXLE_LEN + OVL, dz=axis_z)
    m += axle
    # cap screw bosses: 4 M3 inserts in the cup rim
    for sx in (-1, 1):
        for sy in (-1, 1):
            xa, xb = sorted((sx * (H_HX - 5.2), sx * (H_HX - 0.6)))
            ya, yb = sorted((sy * (H_HY - 4.6), sy * (H_HY - 0.4)))
            m += pl.prism(box(xa, ya, xb, yb), z_top - 10.0, z_top)
    return m, z_bot, z_top


def housing_cap(axis_z, z_top):
    """The plate that closes the cup and clamps the servo's upper tab."""
    outer = pl.rounded_rect(2 * H_HX, 2 * H_HY, 3.0)
    holes = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            xa, xb = sorted((sx * (H_HX - 4.4), sx * (H_HX - 1.4)))
            ya, yb = sorted((sy * (H_HY - 3.4), sy * (H_HY - 1.4)))
            holes.append(box(xa, ya, xb, yb))
    return pl.prism(outer.difference(unary_union(holes)), z_top, z_top + 3.0)


# ------------------------------------------------------------------- arm ----
def arm_profile(w=None, d=None):
    return pl.rounded_rect(w or P.ARM_W, d or P.ARM_D, 4.0)


def arm_tube(z0, z1, clips=True):
    """Closed hollow segment. Printed upright the cavity is a vertical hole,
    so it needs no bridging; walls are ARM_WALL thick."""
    outer = arm_profile()
    inner = outer.buffer(-P.ARM_WALL)
    m = pl.Mesh()
    m += pl.prism(outer, z0, z0 + 2.0)                     # end skin
    m += pl.prism(pl.ring2d(outer, inner), z0 + 2.0 - OVL, z1 - 2.0 + OVL)
    m += pl.prism(outer, z1 - 2.0, z1)                     # end skin
    if clips:
        m += cable_clips(z0 + 4.0, z1 - 4.0)
    return m


def cable_clips(z0, z1):
    """Exterior zip-tie clips on the back face -- the visible service loop
    stays put instead of catching in the joints."""
    span = z1 - z0
    if span < 8.0:                       # tube too short to carry a clip
        return pl.Mesh()
    m = pl.Mesh()
    y = P.ARM_D / 2.0
    w, t = 11.0, 3.2
    n = max(int(span // P.CLIP_PITCH), 1)
    for i in range(n):
        z = z0 + span * (i + 0.5) / n
        bar = box(-w / 2, y - OVL, w / 2, y + t)
        slot = box(-w / 2 - 1, y + 1.0, w / 2 + 1, y + t + 1)
        m += pl.banded(bar, z - 3.0, z + 3.0, [(slot, z - 1.3, z + 1.3)])
    return m


def flare(z0, z1, prof_a, prof_b, seg=64):
    """45-deg-safe loft between two profiles (arm tube <-> yoke / housing)."""
    return pl.loft_solid(pl.resample_ring(prof_a, seg), z0,
                         pl.resample_ring(prof_b, seg), z1)


def hole_y(d, y0, y1, cx, cz, seg=16):
    """openings[] for a bore whose axis runs along Y (used where a part is
    modelled in Z but bolts down through what becomes its world floor)."""
    out, r = [], d / 2.0
    for i in range(seg):
        a0, a1 = math.pi * i / seg, math.pi * (i + 1) / seg
        zh, zl = cz + r * math.cos(a0), cz + r * math.cos(a1)
        w = max(r * math.sin(a0), r * math.sin(a1))
        if zh - zl > 1e-6:
            out.append((box(cx - w, y0, cx + w, y1), zl, zh))
    return out

"""part_band.py — the THENAR BAND: wrist cuff, electronics bay, removable
cap, detachable camera module and the thenar pressure pad.

This is Thenar's own device, not part of Hotaru, so its parameters live here
rather than in params.py. Same rules as every other part in this repo though:
pure shapely 2D booleans extruded into watertight shells, no 3D CSG. One part
is a UNION OF OVERLAPPING SHELLS that the slicer fuses; each shell is closed
on its own.

Frame: +Z runs along the forearm, proximal to distal. XY is the wrist cross
section. +Y is dorsal (back of the hand), so the bay and the camera sit on
the back where they cannot be crushed against a worktop, and the pressure pad
reaches around to the palm on -Y.

The wrist is an ELLIPSE, not a circle. Modelling it round is the classic way
to make a wrist wearable that rocks, digs in at the styloid and reads its
sensors through a moving air gap. Adult wrists run roughly 55 x 42 mm across
the mid carpal region, so the bore is 58 x 46 and the cuff is a C, not a
closed ring: it springs on, and the strap takes up the rest of the range.

The camera is a BAYONET, not a screw. Three lugs, drop in and twist 25
degrees to a hard stop. It is one handed, tool free, and it cannot back out
under vibration the way a thread can, which matters when the whole point of
the device is that somebody wears it while actually working.
"""
from __future__ import annotations

import math
import os
import sys

from shapely import affinity
from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import partlib as pl

OVL = pl.OVL

# ----------------------------------------------------------- parameters ----
WRIST_X, WRIST_Y = 58.0, 46.0        # bore across the carpals, with clearance
CUFF_WALL = 3.4
CUFF_Z0, CUFF_Z1 = 0.0, 30.0
CUFF_GAP = 30.0                      # palmar opening the cuff springs over

BAY_W, BAY_L, BAY_R = 48.0, 34.0, 7.0    # electronics bay footprint
BAY_H = 15.0                             # how far it stands off the cuff
BAY_WALL = 2.4
CAP_T = 4.6                              # cap plate thickness
CAP_LIP = 1.8                            # lip that drops into the bay mouth
CAP_FIT = 0.25                           # per side clearance, printed snap fit

# ---- bayonet ----
# Three lugs pass axially through three entry gaps in a flange, land in a
# groove behind it, and twist until they sit over solid flange. The twist
# angle is not a matter of taste: the lug must clear the gap it came in
# through, and must not reach the next one, so with 3 lugs at 120 deg
#     (ENTRY_SWEEP + LUG_SWEEP) / 2  <  twist  <  120 - (ENTRY_SWEEP + LUG_SWEEP) / 2
# which here is 37 < twist < 83. 60 sits in the middle of that window, so a
# tolerance stack has to be enormous before the joint either jams or lets go.
# audit_band.py checks this rather than trusting the comment.
BAY_MOUNT_D = 26.5                   # socket bore, the stem passes through
BAY_LUG_D = 33.0                     # lug outer diameter
LUG_SWEEP = 34.0                     # arc each lug covers
ENTRY_SWEEP = 40.0                   # arc of each entry gap, lug plus clearance
LUG_T = 2.6
FLANGE_T = 2.5                       # the flange the lugs lock behind
GROOVE_T = 3.5                       # axial room the lugs rotate in
BACK_T = 2.5
SOCKET_D = 8.5                       # FLANGE_T + GROOVE_T + BACK_T
LUG_Z0 = FLANGE_T + 0.45             # lug sits in the groove, off the flange
BAYO_TWIST = 60.0

CAM_W, CAM_H = 26.0, 26.0
CAM_D = 17.0
LENS_D, LENS_OUT = 14.0, 5.5

PAD_X, PAD_Y = 32.0, 24.0            # thenar pad, sized to the muscle mound
PAD_T = 5.0
ARM_W, ARM_T = 12.0, 3.0

SEG = 96

# The camera looks DOWN THE HAND, not at the ceiling. It sits on the distal
# face of the bay and is tilted palmar so the fingers and whatever they are
# holding are in frame. Socket and module both go through _mount(), because
# a bayonet whose two halves are placed by separate literals is a bayonet
# that stops lining up the first time either number is edited.
MOUNT_TILT = 20.0            # degrees toward the palm
MOUNT_Y, MOUNT_Z = 33.0, 29.0


def _mount(m, twist=0.0):
    if twist:
        m.rotate_z(twist)
    m.rotate_x(MOUNT_TILT)
    m.translate(dy=MOUNT_Y, dz=MOUNT_Z)
    return m

# Rendered against a near black page, so the greys are pitched light enough
# to read as machined parts rather than as holes in the background. Cap and
# pad take the brand blue and pink because they are the two parts a visitor
# is meant to notice coming off.
COLOURS = {
    "band-cuff": "#9AA1B0",
    "band-cap":  "#4D17F5",
    "band-cam":  "#5C6273",
    "band-pad":  "#FA9DCD",
}


def ellipse(w, h, seg=SEG):
    """Wrists are elliptical; circle() scaled is the honest profile."""
    return affinity.scale(pl.circle(2.0, seg), xfact=w / 2.0, yfact=h / 2.0)


def _lugs(inner_d, outer_d, sweep, phase=0.0):
    """Three bayonet lugs, as a 2D union of ring sectors.

    Built as a triangle fan clipped by the ring rather than by intersecting
    half planes, because a sector wider than 90 degrees cannot be expressed
    as the intersection of two half planes at all.
    """
    from shapely.geometry import Polygon
    ring = pl.ring2d(pl.circle(outer_d, SEG), pl.circle(inner_d, SEG))
    out = []
    R = outer_d
    for i in range(3):
        a = phase + i * 120.0
        pts = [(0.0, 0.0)]
        for k in range(17):
            t = math.radians(a - sweep / 2.0 + sweep * k / 16.0)
            pts.append((R * math.cos(t), R * math.sin(t)))
        out.append(ring.intersection(Polygon(pts)))
    return unary_union(out)


def lug_ring(twist=0.0):
    """The three lugs on the camera foot. Their ROOT is at the socket bore,
    not at the stem: the 0.4 mm of running clearance the stem needs would
    otherwise hang over the bore with nothing under it, and that sliver is
    lug area carrying no pull out load."""
    g = _lugs(BAY_MOUNT_D, BAY_LUG_D, LUG_SWEEP)
    return affinity.rotate(g, twist, origin=(0, 0)) if twist else g


def entry_ring():
    """The three gaps in the flange the lugs drop through."""
    return _lugs(BAY_MOUNT_D, BAY_LUG_D + 0.9, ENTRY_SWEEP)


def flange_ring():
    """The flange face the lugs lock behind."""
    plate = pl.ring2d(pl.circle(BAY_LUG_D + 5.0, SEG), pl.circle(BAY_MOUNT_D, SEG))
    return plate.difference(entry_ring())


# --------------------------------------------------------------- cuff ----
def cuff():
    """The C shaped wrist chassis, with the bay and the bayonet socket grown
    off its dorsal face as separate overlapping shells."""
    outer = ellipse(WRIST_X + 2 * CUFF_WALL, WRIST_Y + 2 * CUFF_WALL)
    bore = ellipse(WRIST_X, WRIST_Y)
    ring = outer.difference(bore)

    # palmar opening: the cuff springs over the wrist rather than threading on
    gap = box(-CUFF_GAP / 2.0, -(WRIST_Y / 2 + CUFF_WALL + 2), CUFF_GAP / 2.0, 0)
    ring = ring.difference(gap)

    # strap slots, one either side of the opening, angled to follow the cuff
    slots = []
    for sx in (-1, 1):
        s = pl.slot(11.0, 3.2)
        s = affinity.rotate(s, 74.0 * sx)
        s = affinity.translate(s, sx * (WRIST_X / 2 + CUFF_WALL / 2) * 0.86,
                               -WRIST_Y / 2 * 0.70)
        slots.append(s)
    ring = ring.difference(unary_union(slots))

    m = pl.prism(ring, CUFF_Z0, CUFF_Z1)

    # ---- electronics bay, extruded along +Y then laid onto the cuff ----
    bay_out = pl.rounded_rect(BAY_W, BAY_L, BAY_R)
    bay_in = pl.rounded_rect(BAY_W - 2 * BAY_WALL, BAY_L - 2 * BAY_WALL,
                             max(BAY_R - BAY_WALL, 0.6))
    # walls only: the floor is the cuff it sits on, the ceiling is the cap
    walls = pl.prism(bay_out.difference(bay_in), 0.0, BAY_H)
    # a mouth ledge for the cap to land on, one wall thickness down
    ledge = pl.prism(
        bay_in.difference(pl.rounded_rect(BAY_W - 2 * BAY_WALL - 3.0,
                                          BAY_L - 2 * BAY_WALL - 3.0,
                                          max(BAY_R - BAY_WALL - 1.5, 0.6))),
        BAY_H - CAP_LIP - 1.2, BAY_H - CAP_LIP)
    floor = pl.prism(bay_out, -OVL, 1.8)
    bay = pl.Mesh()
    bay += walls
    bay += ledge
    bay += floor
    bay.rotate_x(-90.0)                       # local +Z becomes world +Y
    bay.translate(dy=WRIST_Y / 2 + CUFF_WALL - 1.4, dz=CUFF_Z1 / 2)
    m += bay

    # ---- bayonet socket, on the distal face of the bay ----
    outer = pl.circle(BAY_LUG_D + 5.0, SEG)
    plate = pl.ring2d(outer, pl.circle(BAY_MOUNT_D, SEG))

    sock = pl.Mesh()
    # flange with three entry gaps: this is the face the lugs lock behind
    sock += pl.prism(flange_ring(), 0.0, FLANGE_T)
    # groove: only the outer wall, so a lug inside is free to rotate
    sock += pl.prism(pl.ring2d(outer, pl.circle(BAY_LUG_D + 0.9, SEG)),
                     FLANGE_T, FLANGE_T + GROOVE_T)
    # back wall closes the pocket and gives the lug something to bottom on
    sock += pl.prism(plate, FLANGE_T + GROOVE_T, SOCKET_D)
    _mount(sock)
    m += sock
    return m


# ---------------------------------------------------------------- cap ----
def cap():
    """Removable lid. A lip drops into the bay mouth and a thumb notch on the
    proximal edge gives somewhere to lever it off with a gloved hand."""
    plate = pl.rounded_rect(BAY_W, BAY_L, BAY_R)
    notch = affinity.translate(pl.slot(16.0, 5.0), 0, -BAY_L / 2)
    plate = plate.difference(notch)

    lip = pl.rounded_rect(BAY_W - 2 * BAY_WALL - 2 * CAP_FIT,
                          BAY_L - 2 * BAY_WALL - 2 * CAP_FIT,
                          max(BAY_R - BAY_WALL, 0.6))

    # vents: the bay holds a radio and a battery, so it needs to breathe
    vents = unary_union([affinity.translate(pl.slot(20.0, 2.2), 0, y)
                         for y in (-6.0, 0.0, 6.0)])

    m = pl.Mesh()
    m += pl.prism(plate.difference(vents), 0.0, CAP_T)
    m += pl.prism(lip.difference(vents), CAP_T - OVL, CAP_T + CAP_LIP)
    return m


# --------------------------------------------------------------- camera ----
def camera():
    """The detachable module: body, stepped lens barrel, and the bayonet foot.

    The foot is a stem that passes through the socket bore with the three
    lugs part way along it, so when the module is twisted home the lugs are
    behind the flange and the only way out is to twist back.
    """
    m = pl.Mesh()

    # stem through the socket bore
    m += pl.prism(pl.circle(BAY_MOUNT_D - 0.8, SEG), 0.0, SOCKET_D + OVL)
    # the lugs, sitting in the groove
    m += pl.prism(lug_ring(), LUG_Z0, LUG_Z0 + LUG_T)

    # body starts where the socket ends
    z0 = SOCKET_D
    m += pl.prism(pl.rounded_rect(CAM_W, CAM_H, 6.0), z0 - OVL, z0 + CAM_D)
    # knurl band to twist against, so it is usable with one gloved hand
    m += pl.prism(pl.circle(BAY_LUG_D + 6.0, 24), z0 - OVL, z0 + 3.2)

    # stepped lens barrel, so it reads as optics and not as a peg
    m += pl.prism(pl.circle(LENS_D + 3.0, SEG), z0 + CAM_D - OVL, z0 + CAM_D + 2.0)
    m += pl.prism(pl.circle(LENS_D, SEG), z0 + CAM_D + 2.0 - OVL,
                  z0 + CAM_D + LENS_OUT)
    return m


# ------------------------------------------------------------ thenar pad ----
def pad():
    """The pressure pad and the arm that carries it around to the palm.

    The arm is deliberately thin: it has to flex to sit against a mound that
    changes shape as the thumb opposes, and a rigid arm would lift the pad off
    exactly when the grip it is measuring begins.
    """
    m = pl.Mesh()
    m += pl.prism(ellipse(PAD_X, PAD_Y), 0.0, PAD_T)
    # a raised dome ring so the load lands on the sensor, not the rim
    m += pl.prism(ellipse(PAD_X - 9.0, PAD_Y - 7.0), PAD_T - OVL, PAD_T + 1.6)

    arm = pl.stroke([(0, PAD_Y / 2 - 2.0), (0, PAD_Y / 2 + 26.0)], ARM_W)
    m += pl.prism(arm, 0.0, ARM_T)
    return m


def assembled():
    """Every part transformed into its worn position: cap seated on the bay,
    camera twisted home in the bayonet, pad reaching round to the palm."""
    cu, ca, cm, pd = cuff(), cap(), camera(), pad()

    # cap drops onto the bay mouth, lip pointing down into it
    ca.rotate_x(90.0)
    ca.translate(dy=WRIST_Y / 2 + CUFF_WALL - 1.4 + BAY_H + CAP_T, dz=CUFF_Z1 / 2)

    # camera locked home: same mount, plus the twist to the hard stop
    _mount(cm, twist=BAYO_TWIST)

    # pad swings round the ulnar side to sit on the thenar mound
    pd.rotate_x(-90.0)
    pd.rotate_z(180.0)
    pd.translate(dy=-(WRIST_Y / 2 + CUFF_WALL - 1.0), dz=CUFF_Z1 - 6.0)
    return {"band-cuff": cu, "band-cap": ca, "band-cam": cm, "band-pad": pd}


def build():
    return {
        "band-cuff": cuff(),
        "band-cap": cap(),
        "band-cam": camera(),
        "band-pad": pad(),
    }


def main():
    parts = build()
    ok = True
    for name, m in parts.items():
        rep = pl.validate(m)
        b = m.bounds()
        size = tuple(round(b[i + 3] - b[i], 1) for i in range(3))
        fits = pl.fits_build_plate(m)
        print(f"{name:11s} tris={len(m.F):6d} size={size} "
              f"plate={'ok' if fits else 'TOO BIG'}  {rep}")
        if not fits:
            ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

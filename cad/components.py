"""components.py — visual models of the actual hardware, for fit checking.

These are NOT parts to print. They exist so you can drop the real components
into the assembly and SEE whether they seat: does the USB-C reach the wall,
does the mic capsule line up with its ports, do the header pins clear the
floor, does the servo sit in its cup. Dimensions come from the product
photos and datasheets and are TODO until calipered -- but they are the same
numbers the enclosure is cut from, so if a component looks wrong here it is
wrong in the pocket too.

Each builder returns [(name, Mesh, colour)] in its OWN local frame with a
documented origin. place() moves them into the world.

  esp32_s3()    origin = PCB underside centre, +X = USB-C end
  inmp441()     origin = PCB back face centre, +Z = the mic looks this way
  max98357a()   origin = PCB underside centre
  speaker2040() origin = back face centre, +Z = the cone fires this way
  mg996r()      origin = body centre, +Z = output shaft
  sg90()        same
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
from shapely import affinity
from shapely.geometry import box
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
import partlib as pl

OVL = pl.OVL
C = P.COLORS


def _b(w, h, z0, z1, cx=0.0, cy=0.0, r=0.0):
    prof = pl.rounded_rect(w, h, r) if r else box(-w / 2, -h / 2, w / 2, h / 2)
    return pl.prism(affinity.translate(prof, cx, cy), z0, z1)


def _pin(cx, cy, z0, z1, d=0.64):
    return pl.prism(affinity.translate(box(-d / 2, -d / 2, d / 2, d / 2), cx, cy), z0, z1)


# ------------------------------------------------------- ESP32-S3 N16R8 ----
def esp32_s3():
    """Dual-USB-C ROBODUINO clone. Origin = PCB underside centre, USB-C at +X,
    WROOM module at -X, header pins DOWN."""
    L, W, T = P.ESP_L, P.ESP_W, P.ESP_T
    # corner mounting holes -- real boards have them, and the tub's printed
    # locating pins rise through them now that the M2 screws are gone
    holes = unary_union([affinity.translate(pl.circle(2.4, 16),
                                            sx * (L - 5) / 2, sy * (W - 5) / 2)
                         for sx in (-1, 1) for sy in (-1, 1)])
    prof = pl.rounded_rect(L, W, 1.5).difference(holes)
    pcb = pl.prism(prof, 0.0, T)
    parts = [("esp32-pcb", pcb, C["pcb-black"])]

    m = pl.Mesh()
    mod_x = -L / 2 + P.ESP_MOD_W / 2 + 2.0
    m += _b(P.ESP_MOD_W, P.ESP_MOD_L, T - OVL, T + P.ESP_MOD_H, mod_x, 0.0)
    m += _b(P.ESP_ANT_L, P.ESP_MOD_L * 0.62, T - OVL, T + 1.0,
            -L / 2 - P.ESP_ANT_L / 2 + 1.0, 0.0)          # antenna overhang
    parts.append(("esp32-module", m, C["shield"]))

    usb = pl.Mesh()
    for sy in (-1, 1):
        usb += _b(P.ESP_USB_D, P.ESP_USB_W, T - OVL, T + P.ESP_USB_H,
                  L / 2 - P.ESP_USB_D / 2 + 1.2, sy * P.ESP_USB_OFF, r=1.2)
    parts.append(("esp32-usb", usb, C["silver"]))

    bits = pl.Mesh()
    for bx, by in ((-2.0, 9.0), (6.0, 9.0)):
        bits += _b(P.ESP_BTN, P.ESP_BTN, T - OVL, T + 3.4, bx, by, r=0.6)
    parts.append(("esp32-buttons", bits, C["silver"]))
    parts.append(("esp32-led", _b(5.0, 5.0, T - OVL, T + 1.6, -6.0, -2.0),
                  C["white"]))

    hdr, pins = pl.Mesh(), pl.Mesh()
    span = (P.ESP_HDR_N - 1) * P.ESP_HDR_PITCH
    for sy in (-1, 1):
        hy = sy * (W / 2 - 1.6)
        hdr += _b(span + 2.6, 2.6, -P.ESP_HDR_BODY, 0.0, 0.0, hy)
        for i in range(P.ESP_HDR_N):
            pins += _pin(-span / 2 + i * P.ESP_HDR_PITCH, hy,
                         -P.ESP_PIN_DROP, 0.0)
    parts.append(("esp32-headers", hdr, C["gold"]))
    parts.append(("esp32-pins", pins, C["silver"]))
    return parts


# ------------------------------------------------------------- INMP441 ----
def inmp441():
    """Round board with two flats. Origin = PCB BACK face centre; the mic
    package sits at +Z, i.e. +Z is the direction it listens."""
    d, t = P.MIC_D, P.MIC_T
    prof = pl.circle(d, 64).intersection(box(-P.MIC_FLAT / 2, -d, P.MIC_FLAT / 2, d))
    ring = pl.ring2d(pl.circle(d - 1.4, 64), pl.circle(d - 3.6, 64)).intersection(
        box(-P.MIC_FLAT / 2, -d, P.MIC_FLAT / 2, d))
    parts = [("mic-pcb", pl.prism(prof, 0.0, t), C["pcb-black"]),
             ("mic-ring", pl.prism(ring, t - OVL, t + 0.05), C["gold"]),
             ("mic-pkg", _b(*P.MIC_PKG[:2], t - OVL, t + P.MIC_PKG[2]), C["silver"])]
    pins = pl.Mesh()
    nx, ny = P.MIC_PINS
    for i in range(nx):
        for j in range(ny):
            pins += _pin((i - (nx - 1) / 2) * 2.54, (j - (ny - 1) / 2) * 2.54,
                         -6.0, 0.0)
    parts.append(("mic-pins", pins, C["silver"]))
    return parts


# ---------------------------------------------------------- MAX98357A ----
def max98357a():
    """Purple breakout, green screw terminal on top, 7 pins down one edge."""
    L, W, T = P.AMP_L, P.AMP_W, P.AMP_T
    holes = unary_union([affinity.translate(pl.circle(2.4, 16),
                                            sx * 7.5, oy)
                         for sx in (-1, 1) for oy in (6.0, -2.5)])
    prof = pl.rounded_rect(L, W, 1.0).difference(holes)
    parts = [("amp-pcb", pl.prism(prof, 0.0, T), C["pcb-purple"])]
    tl, tw, th = P.AMP_TERM
    parts.append(("amp-terminal", _b(tl, tw, T - OVL, T + th, -1.5, W / 2 - tw / 2 - 1.5),
                  C["terminal"]))
    holes = pl.Mesh()
    for sx in (-1, 1):
        holes += pl.prism(pl.ring2d(
            affinity.translate(pl.circle(4.2), sx * (L / 2 - 3.0), W / 2 - 3.0),
            affinity.translate(pl.circle(2.6), sx * (L / 2 - 3.0), W / 2 - 3.0)),
            T - OVL, T + 0.05)
    parts.append(("amp-pads", holes, C["gold"]))
    hdr, pins = pl.Mesh(), pl.Mesh()
    span = (P.AMP_PINS - 1) * 2.54
    hy = -W / 2 + 1.6
    hdr += _b(span + 2.6, 2.6, -2.5, 0.0, 0.0, hy)
    for i in range(P.AMP_PINS):
        pins += _pin(-span / 2 + i * 2.54, hy, -6.0, 0.0)
    parts.append(("amp-headers", hdr, C["gold"]))
    parts.append(("amp-pins", pins, C["silver"]))
    return parts


# ------------------------------------------------------------ speaker ----
def speaker2040():
    """2040 rectangular driver. Origin = BACK face centre, cone fires +Z."""
    L, W, T = P.SPK_L, P.SPK_W, P.SPK_T
    frame = pl.rounded_rect(L, W, 1.5)
    cone = affinity.scale(pl.circle(P.SPK_CONE[0], 48),
                          1.0, P.SPK_CONE[1] / P.SPK_CONE[0])
    parts = [("spk-frame", pl.prism(frame.difference(cone), 0.0, T), C["speaker"]),
             ("spk-back", pl.prism(frame, 0.0, 1.2), C["speaker"]),
             ("spk-cone", pl.prism(cone, 1.2 - OVL, T - 1.2), C["cone"]),
             ("spk-dome", pl.revolve_shell(
                 T - 1.2 - OVL, T + 1.4,
                 lambda z: max(11.0 * math.sqrt(max(1 - ((z - (T - 1.2)) / 2.6) ** 2,
                                                    1e-4)), 0.6), 0.25, steps=10, seg=48),
              C["cone"])]
    # Wires leave the BACK of the driver (-Z), into the enclosure. Running
    # them +Z sent them straight out through the grille slots.
    wires = pl.Mesh()
    for sy in (-1, 1):
        wires += pl.prism(affinity.translate(pl.circle(P.SPK_WIRE_D, 12),
                                             L / 2 - 4.0, sy * 2.0), -9.0, 0.6)
    parts.append(("spk-wires", wires, C["wire-red"]))
    return parts


# -------------------------------------------------------------- servos ----
def _servo(bl, bw, bh, tab_span, tab_t, tab_z, shaft_off, shaft_d, boss_d,
           boss_h, shaft_l, tag, label=True, col="servo"):
    """Generic hobby servo. Origin = body centre, output shaft +Z, shaft
    offset `shaft_off` from the -X end of the body."""
    sx = -bl / 2 + shaft_off
    parts = [(f"{tag}-body", _b(bl, bw, 0.0, bh, r=1.2), C[col]),
             (f"{tag}-tabs", _b(tab_span, bw, tab_z, tab_z + tab_t, r=1.5), C[col])]
    if label:
        parts.append((f"{tag}-label", _b(bl - 4.0, bw - 3.0, bh - P.MG_LABEL_H, bh),
                      C["label"]))
    m = pl.Mesh()
    m += pl.prism(affinity.translate(pl.circle(boss_d), sx, 0.0), bh - OVL, bh + boss_h)
    m += pl.prism(affinity.translate(pl.circle(boss_d * 0.55), -sx * 0.9, 0.0),
                  bh - OVL, bh + boss_h * 0.5)
    parts.append((f"{tag}-boss", m, C[col]))
    parts.append((f"{tag}-spline",
                  pl.prism(affinity.translate(pl.circle(shaft_d), sx, 0.0),
                           bh + boss_h - OVL, bh + boss_h + shaft_l), C["brass"]))
    wire = pl.Mesh()
    for i, col in enumerate(("wire-black", "wire-red", "cone")):
        wire += pl.prism(affinity.translate(pl.circle(1.5, 10),
                                            -bl / 2 - 2.0, (i - 1) * 1.7),
                         bh * 0.35, bh * 0.35 + 1.4)
    parts.append((f"{tag}-wire", wire, C["wire-red"]))
    return parts


def mg996r():
    return _servo(P.MG_L, P.MG_W, P.MG_H, P.MG_TAB_SPAN, P.MG_TAB_T, P.MG_TAB_Z,
                  P.MG_SHAFT_OFF, P.MG_SHAFT_D, P.MG_BOSS_D, P.MG_BOSS_H,
                  P.MG_SHAFT_TOP - P.MG_H - P.MG_BOSS_H, "mg996r")


def sg90():
    return _servo(P.SG_L, P.SG_W, P.SG_H, P.SG_TAB_SPAN, P.SG_TAB_T, P.SG_TAB_Z,
                  P.SG_SHAFT_OFF, P.SG_SHAFT_D, P.SG_HORN_HUB_D, 2.5, 3.5,
                  "sg90", label=False, col="servo2")


def mx_switch():
    """Cherry MX, origin at the PLATE TOP: upper housing sits on the plate,
    body and pins hang below, stem stands proud. The cap goes over the stem."""
    up = P.MX_UPPER_SQ
    parts = [("mx-upper", _b(up, up, 0.0, P.MX_UPPER_H, r=0.6), C["pcb-black"]),
             # the body NECKS at the plate: 15 square below, 13.9 through
             # the 14.1 cutout -- that neck is what the clips grip
             ("mx-body", _b(P.MX_BODY_SQ - 1.0, P.MX_BODY_SQ - 1.0,
                            -P.MX_BODY_DROP, -P.MX_PLATE_T, r=0.6), C["white"]),
             ("mx-neck", _b(P.MX_CUT - 0.2, P.MX_CUT - 0.2,
                            -P.MX_PLATE_T, 0.0, r=0.4), C["white"])]
    stem = P.MX_UPPER_H
    parts.append(("mx-stem", _b(P.MX_STEM_SQ, P.MX_STEM_SQ, stem,
                                stem + P.MX_STEM_UP, r=0.4), C["keycap"]))
    for tag, sx in (("l", -2.5), ("r", 2.5)):
        parts.append((f"mx-pins-{tag}",
                      _b(0.9, 0.5, -P.MX_BODY_DROP - P.MX_PIN_DROP,
                         -P.MX_BODY_DROP, cx=sx), C["metal"]))
    return parts


def place(parts, rx=0.0, ry=0.0, rz=0.0, dx=0.0, dy=0.0, dz=0.0, tag=""):
    out = []
    for n, m, c in parts:
        q = m.copy()
        if rz:
            q.rotate_z(rz)
        if ry:
            q.rotate_y(ry)
        if rx:
            q.rotate_x(rx)
        q.translate(dx, dy, dz)
        out.append((f"{n}{tag}", q, c))
    return out


if __name__ == "__main__":
    ok = True
    for fn in (esp32_s3, inmp441, max98357a, speaker2040, mg996r, sg90):
        items = fn()
        tris = sum(len(m.F) for _n, m, _c in items)
        bad = [n for n, m, _c in items if not pl.validate(m)["watertight"]]
        lo = np.array([min(v[i] for _n, m, _c in items for v in m.V) for i in range(3)])
        hi = np.array([max(v[i] for _n, m, _c in items for v in m.V) for i in range(3)])
        print(f"{fn.__name__:12s} {len(items)} parts  {tris:5d} tris  "
              f"bbox {tuple(round(float(v),1) for v in (hi-lo))}  "
              f"{'OK' if not bad else 'BAD: ' + str(bad)}")
        ok &= not bad
    sys.exit(0 if ok else 1)


# --------------------------------------------------- per-part placement ----
def for_part(name):
    """Components that belong to one printed part, in THAT PART'S OWN MODEL
    FRAME (before any print-orientation flip or drop).

    Lets a print plate show its hardware in place -- so 'plate 3, arms' can
    be checked for servo fit without leaving the plate view. Preview only:
    these never reach the STL.
    """
    if name == "base":
        wall_in = -P.BASE_D / 2 + P.WALL_STRUCT
        # Board and amp both go in UPSIDE DOWN (rx=180) so their header pins
        # face up where you can reach them. That drops the board's USB-C and
        # module below the PCB, and the amp's screw terminal below its PCB --
        # which is why the board sits at Z=12 and the amp on a 12 mm frame.
        esp = (place(esp32_s3(), rx=180.0, dx=P.ESP_CTR[0], dy=P.ESP_CTR[1],
                     dz=P.ESP_Z + P.ESP_T) if P.ESP_FLIP else
               place(esp32_s3(), dx=P.ESP_CTR[0], dy=P.ESP_CTR[1], dz=P.ESP_Z))
        amp = (place(max98357a(), rx=180.0, dx=P.AMP_CTR[0], dy=P.AMP_CTR[1],
                     dz=P.FLOOR + P.AMP_STANDOFF + P.AMP_T) if P.AMP_FLIP else
               place(max98357a(), dx=P.AMP_CTR[0], dy=P.AMP_CTR[1],
                     dz=P.FLOOR + P.AMP_STANDOFF))
        return (esp + amp
                + place(inmp441(), rx=90.0, dx=P.MIC_CTR_X,
                        dy=wall_in + P.MIC_POCKET_D + 2.4, dz=P.MIC_CTR_Z)
                + place(speaker2040(), ry=-90.0, rx=90.0, dx=P.SPK_CTR_X,
                        dy=P.SPK_CTR_Y, dz=P.SPK_CTR_Z))

    def _mg(axis_z, tag):
        return place(mg996r(), ry=90.0, dx=-P.MG_H / 2,
                     dz=axis_z - (P.MG_L / 2 - P.MG_SHAFT_OFF), tag=tag)

    if name == "lid":
        import part_keycap as PKC
        z = P.LID_Z1
        out = place(mx_switch(), dx=P.MX_CTR[0], dy=P.MX_CTR[1], dz=z)
        # Cap underside sits KEYCAP_GAP above the upper housing -- that gap is
        # the key's travel. Deriving it from the stem top instead (as this
        # first did) happened to land the cap flat on the housing with zero
        # clearance, i.e. a key that is already bottomed out.
        cap_z = z + P.MX_UPPER_H + P.KEYCAP_GAP
        for n, m, c in PKC.build():
            out.append((n, m.copy().translate(dx=P.MX_CTR[0], dy=P.MX_CTR[1],
                                              dz=cap_z), c))
        return out
    if name == "base-joint":
        import part_base_joint as PBJ
        return _mg(PBJ.AXIS, "-base")
    if name == "arm-lower":
        return _mg(P.YOKE_BELOW + P.ARM_LOWER_L, "-shoulder")
    if name == "arm-upper":
        return _mg(P.YOKE_BELOW + P.ARM_UPPER_L, "-elbow")
    if name == "arm-fore":
        return place(sg90(), ry=90.0, dx=-P.SG_H / 2,
                     dz=(P.YOKE_BELOW + P.ARM_FORE_L)
                     - (P.SG_L / 2 - P.SG_SHAFT_OFF), tag="-head")
    return []


# ---------------------------------------------------------- WS2812 ring ----
def ws2812_ring():
    """The LED ring, as sold: an annular PCB with the LEDs on ONE face and
    the four solder pads on the other. Origin = PCB back-face centre, LEDs
    at +Z. Pads at -Z, grouped near one edge the way the common 12-LED
    boards put them: PWR, GND, IN, OUT."""
    from shapely import affinity as _aff
    import math as _math
    n_led = 12
    pcb = pl.ring2d(pl.circle(P.RING_OD, 96), pl.circle(P.RING_ID, 96))
    parts = [("ring-pcb", pl.prism(pcb, 0.0, P.RING_T), C["pcb-black"])]
    led = pl.Mesh()
    r_led = (P.RING_OD + P.RING_ID) / 4.0
    for i in range(n_led):
        a = 2 * _math.pi * i / n_led
        led += pl.prism(_aff.translate(box(-2.5, -2.5, 2.5, 2.5),
                                       r_led * _math.cos(a),
                                       r_led * _math.sin(a)),
                        P.RING_T - OVL, P.RING_T + 1.6)
    parts.append(("ring-leds", led, C["ring"]))
    pads = pl.Mesh()
    for i in range(4):
        a = _math.radians(258 + 8 * i)
        pads += pl.prism(_aff.translate(box(-1.1, -1.1, 1.1, 1.1),
                                        r_led * _math.cos(a),
                                        r_led * _math.sin(a)),
                         -0.3, 0.0 + OVL)
    parts.append(("ring-pads", pads, C["gold"]))
    return parts


# ------------------------------------------------- stock MG996R horn ------
def mg996r_horn():
    """The black nylon double arm that ships with the servo.

    Modelled as a COMPONENT, not a printed part: it is bought hardware, so
    it belongs with the servos and the boards in the audits that ask
    whether real things fit, not on a build plate.

    Origin = the underside of the arm plate, hub rising at +Z the way it
    sits on the spline. Arms lie along X."""
    from shapely import affinity as _aff
    arm = pl.rounded_rect(P.SHORN_L, P.SHORN_W, P.SHORN_W / 2.0)
    parts = [("shorn-arm", pl.prism(arm, 0.0, P.SHORN_T), C["pcb-black"])]
    hub = pl.circle(P.SHORN_HUB_D, 48)
    parts.append(("shorn-hub",
                  pl.prism(hub, P.SHORN_T - OVL, P.SHORN_T + P.SHORN_HUB_H),
                  C["pcb-black"]))
    # the six holes per arm are cosmetic here: nothing bolts through them,
    # the pocket carries the torque. Cut them so the render reads as the
    # real part rather than a black lozenge.
    holes = []
    for s_ in (-1, 1):
        for k in range(6):
            holes.append(_aff.translate(pl.circle(2.0, 12),
                                        s_ * (9.0 + k * 3.5), 0.0))
    return parts

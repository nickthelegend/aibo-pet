"""export_rig_v2.py — the HOTARU 2.0 hero rig for web/app.js.

The landing page animates a lamp: it needs each moving group in its OWN
frame with the joint pivot at the origin, plus the numbers describing how
the groups chain. v1's export_web.py did this for the v1 arm; this does it
for 2.0, and 2.0 differs in one structural way that the rig format has to
learn:

    v1's "base" joint was a TILT about X, so all four joints were one
    cumulative rotX chain. 2.0's base is a PAN about Z. It cannot be folded
    into the same chain, so the rig now names a `panParts` group and app.js
    applies rotZ(base) as an OUTER transform to everything that turns with
    the disc.

Segment split, and why each part sits where it does -- a servo's CASE moves
with the link it is bolted into, its HORN moves with the link it drives:

    static     tub, boards, MX + keycap, the pan servo's case, disc keys
    panParts   disc, pan horn, tower, the shoulder servo's case
    seg0       link1 + the shoulder HORN + the elbow servo's case   (L1)
    seg1       link2 + the elbow HORN + head block + head servo case (L2)
    seg2       the head HORN and its glued cap -- these turn with the cone
    shade      cone + cap + LED ring

    .venv/bin/python cad/export_rig_v2.py
        -> web/hotaru2-rig.glb + web/rig2.json
"""
from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v2_assembly as A
import v2_parts as V
from export_web import WEB, weld, write_glb

STATIC = ("v2-tub", "mx-", "v2-keycap", "esp32-", "amp-", "mic-", "spk-",
          "pan-", "v2-disckey")
PAN = ("v2-disc", "horn-pan", "v2-tower", "sh-")
SEG0 = ("horn-shoulder", "v2-link1", "v2-screw-elbow", "el-")
SEG1 = ("horn-elbow", "v2-link2", "v2-trimcap", "v2-head", "hd-")
SEG2 = ("horn-head", "v2-caphead")
SHADE = ("shade", "v2-conecap", "ring-")


def classify(n):
    # order matters: "v2-disc" is a prefix of "v2-disckey", and
    # "v2-screw-elbow" must be tested before the bare "v2-screw"
    if n.startswith("v2-disckey"):
        return "static"
    for grp, pres in (("shade", SHADE), ("seg2", SEG2), ("seg1", SEG1),
                      ("seg0", SEG0), ("pan", PAN), ("static", STATIC)):
        if n.startswith(pres):
            return grp
    if n == "v2-screw":
        return "seg0"
    return None


def main():
    os.makedirs(WEB, exist_ok=True)
    z_sh = V.DISC_Z0 + V.DISC_T + V.TWR_AXIS_Z
    z_el = z_sh + V.L1
    z_hd = z_el + V.L2
    origin = {"static": 0.0, "pan": 0.0,
              "seg0": z_sh, "seg1": z_el, "seg2": z_hd, "shade": z_hd}

    items, groups = [], {k: [] for k in
                         ("static", "pan", "seg0", "seg1", "seg2", "shade")}
    unknown = []
    for n, m, c in A.world_items():
        g = classify(n)
        if g is None:
            unknown.append(n)
            continue
        q = m.copy()
        q.translate(dz=-origin[g])
        if g == "shade":
            # app.js poses the cone as trans(p) . rotX(cum + 180), exactly as
            # the assembly builds it, so the local mesh carries the inverse
            q.rotate_x(180.0)
        items.append((n, q, c))
        groups[g].append(n)

    # A part silently dropped here is a part missing from the hero. The v1
    # parts viewer shipped for weeks listing 13 of 22 for exactly this
    # reason, so this is an error, not a warning.
    if unknown:
        raise SystemExit(f"unclassified parts, rig would drop them: {unknown}")
    for g, names in groups.items():
        if not names:
            raise SystemExit(f"group {g} is empty -- the split is wrong")

    def split(names):
        """parts vs servo, so app.js's existing concat() keeps working"""
        pre = ("pan-", "sh-", "el-", "hd-")
        return ([n for n in names if not n.startswith(pre)],
                [n for n in names if n.startswith(pre)])

    segs = []
    for g, length in (("seg0", V.L1), ("seg1", V.L2), ("seg2", 0.0)):
        p, s = split(groups[g])
        segs.append({"parts": p, "servo": s, "length": length})

    rig = {
        "static": groups["static"],
        "panParts": groups["pan"],
        "loose": [],
        "segments": segs,
        "shade": "shade",
        "shadeParts": groups["shade"],
        "pivot": [0.0, 0.0, z_sh],
        # A neutral that reads as a desk lamp rather than a flagpole: the
        # shoulder leans the arm out, the elbow folds it back over the base,
        # and the head tips the cone TOWARD the viewer so the LED ring is
        # visible through the cap's slot. Chosen by eye in the browser --
        # positive rotX tips away from the camera, so the head sits low.
        "neutral": {"base": 0.0, "shoulder": 38.0, "elbow": -62.0,
                    "head": 8.0},
        # Off-neutral travel. The motion audit clears every joint for a full
        # 180 (165 at the head), so these are expressive limits, not
        # mechanical ones -- kept well inside what the servos can reach.
        "range": {"base": [-60.0, 60.0], "shoulder": [-30.0, 30.0],
                  "elbow": [-30.0, 30.0], "head": [-38.0, 38.0]},
    }

    packed = []
    tris = 0
    for n, m, c in items:
        v, f = weld(m)
        tris += len(f) // 3
        packed.append((n, v, f, c))
    write_glb(os.path.join(WEB, "hotaru2-rig.glb"), packed)
    with open(os.path.join(WEB, "rig2.json"), "w") as fh:
        json.dump(rig, fh, indent=1)

    mb = os.path.getsize(os.path.join(WEB, "hotaru2-rig.glb")) / 1e6
    print(f"web/hotaru2-rig.glb  {len(packed)} nodes, {tris:,} tris, {mb:.1f} MB")
    for g in ("static", "pan", "seg0", "seg1", "seg2", "shade"):
        print(f"  {g:8s} {len(groups[g]):3d}  {', '.join(groups[g][:4])}"
              f"{' ...' if len(groups[g]) > 4 else ''}")
    print(f"  pivot z {z_sh:.1f}, L1 {V.L1}, L2 {V.L2}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

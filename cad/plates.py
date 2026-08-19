"""plates.py — pack the parts into print batches for a 180x180 bed.

Shelf-packs every part by footprint into as few plates as fit, keeping a
margin off the bed edge and a gap between parts, then writes one merged STL
and one GLB per plate plus a report. Parts are already in their print
orientation; this only slides them around in XY.

Batches are also grouped by INTENT, so a plate is a thing you actually want
to run:

  plate-1-test      print this first: spline test + one horn. 15 minutes.
  plate-2-base      the big round base, alone (it owns most of the bed)
  plate-3a/b/c      one arm segment each, standing
  plate-4-head      cone + lid
  plate-5-small     caps, horns, joint

    .venv/bin/python cad/plates.py
"""
from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly
import params as P
import partlib as pl

EXPORTS = assembly.EXPORTS
BED = P.PLATE
MARGIN = 6.0        # keep off the bed edge (skirt/brim room)
GAP = 4.0           # between parts, minimum
# Tall parts get more. 4 mm is fine between two 3 mm caps and mean between
# two 154 mm towers on a bedslinger: the gantry travels through that gap on
# every layer, and the parts themselves are swinging back and forth. Scaled
# off the taller of the two neighbours.
GAP_PER_MM_TALL = 0.05
GAP_MAX = 12.0


def gap_for(h):
    return min(max(GAP, h * GAP_PER_MM_TALL), GAP_MAX)

# Grouping is deliberate, not alphabetical -- see the module docstring.
BATCHES = [
    ("1-test", "print FIRST -- verifies the printed spline before you commit",
     ["spline-test", "horn-mg996r"]),
    ("2-base", "the round tub; it owns the bed on its own", ["base"]),
    ("2b-shoulder", "the shoulder ring -- goes on AFTER the electronics",
     ["shoulder"]),
    # One segment per plate, on purpose. They fit together comfortably, but
    # these are 154 mm towers on a bedslinger: three of them are three
    # chances for a knock-over to take the whole 8-hour run with it, and you
    # cannot restart one without restarting all three.
    ("3a-arm-lower", "arm segment 1 of 3 -- standing on its yoke",
     ["arm-lower"]),
    ("3b-arm-upper", "arm segment 2 of 3 -- standing on its yoke",
     ["arm-upper"]),
    ("3c-arm-fore", "arm segment 3 of 3 -- standing on its yoke",
     ["arm-fore"]),
    ("4-head", "cone (mouth down) + lid (top face down)", ["shade", "lid"]),
    ("5-small", "everything that is quick", ["base-joint", "cap-base", "cap-shoulder",
                                             "cap-elbow", "cap-head", "horn-sg90",
                                             "horn-adapter", "spk-clamp", "mic-tab",
                                             "keycap", "esp-tab", "amp-tab"]
     + ["yoke-screw"] * 4),          # one per joint: base, shoulder, elbow, head
]


def _footprint(mesh):
    x0, y0, _z0, x1, y1, _z1 = mesh.bounds()
    return x1 - x0, y1 - y0


def shelf_pack(items):
    """[(name, mesh)] -> [[(name, mesh, dx, dy)]], one list per plate."""
    usable = BED - 2 * MARGIN
    todo = sorted(items, key=lambda it: -_footprint(it[1])[1])
    plates, cur, shelf_y, shelf_h, cx = [], [], MARGIN, 0.0, MARGIN
    shelf_tall = 0.0
    for name, mesh in todo:
        w, h = _footprint(mesh)
        if w > usable or h > usable:
            raise ValueError(f"{name} ({w:.0f}x{h:.0f}) does not fit the bed")
        if cx + w > MARGIN + usable:                      # next shelf
            shelf_y += shelf_h + gap_for(shelf_tall)
            shelf_h, cx, shelf_tall = 0.0, MARGIN, 0.0
        if shelf_y + h > MARGIN + usable:                 # next plate
            plates.append(cur)
            cur, shelf_y, shelf_h, cx, shelf_tall = [], MARGIN, 0.0, MARGIN, 0.0
        b = mesh.bounds()
        cur.append((name, mesh, cx - b[0], shelf_y - b[1]))
        cx += w + gap_for(b[5] - b[2])
        shelf_h = max(shelf_h, h)
        shelf_tall = max(shelf_tall, b[5] - b[2])
    if cur:
        plates.append(cur)
    return plates


def main():
    built = dict(assembly.print_items())
    colors = {n: c for n, _m, c in
              (assembly.part_base.build() + assembly.part_lid.build()
               + assembly.part_base_joint.build() + assembly.part_arms.build()
               + assembly.part_head.build() + assembly.part_horn.build()
               + assembly.part_retainers.build()
               + assembly.part_shoulder.build()
               + assembly.part_keycap.build()
               + assembly.part_screw.build())}
    report, n_plates = [], 0
    print(f"bed {BED:.0f}x{BED:.0f}, margin {MARGIN}, gap {GAP}\n")
    for tag, why, names in BATCHES:
        missing = [n for n in names if n not in built]
        if missing:
            raise KeyError(f"batch {tag}: unknown parts {missing}")
        for i, placed in enumerate(shelf_pack([(n, built[n]) for n in names])):
            n_plates += 1
            label = f"plate-{tag}" + (f"-{i+1}" if i else "")
            merged, items = pl.Mesh(), []
            for name, mesh, dx, dy in placed:
                m = mesh.copy().translate(dx=dx, dy=dy)
                merged += m                       # STL: printed parts ONLY
                items.append((name, m, colors.get(name, "#CCCCCC")))
                # GLB also carries this part's hardware, moved with it, so
                # every plate can be checked for fit without leaving the view
                for cn, cm, cc in assembly.components_in_print_pose(name):
                    items.append((cn, cm.copy().translate(dx=dx, dy=dy), cc))
            b = merged.bounds()
            pl.stl_write(os.path.join(EXPORTS, f"{label}.stl"), merged)
            pl.glb_write(os.path.join(EXPORTS, f"{label}.glb"), items)
            used = (b[3] - b[0], b[4] - b[1], b[5] - b[2])
            ok = used[0] <= BED and used[1] <= BED
            n_comp = len(items) - len(placed)
            print(f"{label:16s} {len(placed)} part(s)"
                  + (f" +{n_comp} comp" if n_comp else "        ") + "  "
                  f"{used[0]:5.0f} x {used[1]:5.0f} x {used[2]:5.0f} mm  "
                  f"{'OK' if ok else 'OVERFLOW'}   {why if not i else ''}")
            for name, _m, _dx, _dy in placed:
                print(f"                   - {name}")
            report.append({"plate": label, "why": why,
                           "parts": [p[0] for p in placed],
                           "used_mm": [round(float(v), 1) for v in used],
                           "fits_bed": bool(ok)})
    with open(os.path.join(EXPORTS, "PLATES.json"), "w") as fh:
        json.dump({"bed_mm": BED, "margin_mm": MARGIN, "gap_mm": GAP,
                   "plates": report}, fh, indent=2)
    print(f"\n{n_plates} plates -> exports/plate-*.stl (+ .glb), PLATES.json")
    return 0 if all(r["fits_bed"] for r in report) else 1


if __name__ == "__main__":
    sys.exit(main())

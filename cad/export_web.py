"""export_web.py — the rig the landing page animates.

exports/hotaru-assembled.glb bakes the arm at ONE pose: every segment is
already rotated and translated into place, so there is nothing left to drive.
Fine for looking at, useless for moving.

This writes the same parts with the moving ones left in their OWN frames --
each segment sitting at its yoke pivot, pointing +Z -- plus the numbers that
describe how they chain together. The page rebuilds the pose every frame from
four joint angles, which is exactly what assembly.world_items() does once at
build time.

    .venv/bin/python cad/export_web.py   ->  web/hotaru-rig.glb + web/rig.json
"""
from __future__ import annotations

import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

import params as P
import part_arms
import part_base
import part_base_joint
import part_head
import part_keycap
import part_lid
import part_shoulder
import partlib as pl
from components import for_part

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.normpath(os.path.join(HERE, "..", "web"))


def weld(mesh):
    """Weld on POSITION only, dropping normals entirely.

    partlib's glb_write explodes every mesh to a triangle soup -- three unique
    vertices per face, plus a normal each -- which is 84 bytes a triangle and
    put this rig at 14.5 MB. Fine for a local model viewer, far too heavy for
    a page someone loads over the network.

    The page computes FLAT normals in the fragment shader from screen-space
    derivatives, so the file does not have to carry them, and without normals
    a vertex is just a position: a cube corner welds from six copies to one.
    """
    out, index, faces = [], {}, []
    for f in mesh.F:
        tri = []
        for i in f:
            v = mesh.V[i]
            key = (round(v[0], 4), round(v[1], 4), round(v[2], 4))
            j = index.get(key)
            if j is None:
                j = index[key] = len(out)
                out.append(v)
            tri.append(j)
        if len(set(tri)) == 3:                 # drop anything degenerate
            faces.append(tuple(tri))
    return out, faces


def write_glb(path, items):
    """Indexed GLB: POSITION + indices, no NORMAL. uint16 where it fits."""
    import struct

    bin_chunk = bytearray()
    views, accessors, meshes, nodes, materials = [], [], [], [], []

    def add_view(blob, target):
        views.append({"buffer": 0, "byteOffset": len(bin_chunk),
                      "byteLength": len(blob), "target": target})
        bin_chunk.extend(blob)
        bin_chunk.extend(b"\0" * (-len(blob) % 4))
        return len(views) - 1

    for i, (name, verts, faces, hexcol) in enumerate(items):
        pos = np.asarray(verts, dtype=np.float32)
        flat = np.asarray(faces, dtype=np.uint32).reshape(-1)
        small = len(pos) < 65536
        idx = flat.astype(np.uint16) if small else flat
        vp = add_view(pos.tobytes(), 34962)
        vi = add_view(idx.tobytes(), 34963)
        accessors += [
            {"bufferView": vp, "componentType": 5126, "count": len(pos),
             "type": "VEC3",
             "min": [float(x) for x in pos.min(axis=0)],
             "max": [float(x) for x in pos.max(axis=0)]},
            {"bufferView": vi, "componentType": 5123 if small else 5125,
             "count": len(idx), "type": "SCALAR"},
        ]
        rgb = [pl._srgb_to_linear(int(hexcol[j:j + 2], 16)) for j in (1, 3, 5)]
        materials.append({"name": f"{name}-mat", "pbrMetallicRoughness": {
            "baseColorFactor": [*rgb, 1.0]}})
        meshes.append({"name": name, "primitives": [{
            "attributes": {"POSITION": 2 * i}, "indices": 2 * i + 1,
            "material": i}]})
        nodes.append({"name": name, "mesh": i})

    js = {"asset": {"version": "2.0", "generator": "hotaru export_web"},
          "scene": 0, "scenes": [{"nodes": list(range(len(nodes)))}],
          "nodes": nodes, "meshes": meshes, "materials": materials,
          "accessors": accessors, "bufferViews": views,
          "buffers": [{"byteLength": len(bin_chunk)}]}
    jb = json.dumps(js, separators=(",", ":")).encode()
    jb += b" " * (-len(jb) % 4)
    total = 12 + 8 + len(jb) + 8 + len(bin_chunk)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total))
        f.write(struct.pack("<II", len(jb), 0x4E4F534A)); f.write(jb)
        f.write(struct.pack("<II", len(bin_chunk), 0x004E4942)); f.write(bin_chunk)


# Which parts move with which joint. Everything else is bolted to the base.
SEGMENTS = [("arm-lower", "cap-shoulder", P.ARM_LOWER_L),
            ("arm-upper", "cap-elbow", P.ARM_UPPER_L),
            ("arm-fore", "cap-head", P.ARM_FORE_L)]


def main():
    os.makedirs(WEB, exist_ok=True)
    items, rig = [], {}

    # ---- static: everything from the floor up to the base joint's cup ----
    static = (part_base.build() + part_shoulder.build() + part_lid.build()
              + [(n, part_base_joint.to_world(m), c)
                 for n, m, c in part_base_joint.build()])
    for n, m, c in static:
        items.append((n, m, c))
    for n, m, c in for_part("lid"):          # MX switch + keycap
        items.append((n, m, c))
    for n, m, c in for_part("base"):         # boards, speaker, mic
        items.append((n, m, c))
    rig["static"] = [n for n, _m, _c in items]

    # ---- moving: each segment in its own frame, pivot at the origin ----
    arms = {n: (m, c) for n, m, c in part_arms.build()}
    seg_meta = []
    for i, (name, capname, length) in enumerate(SEGMENTS):
        for nm in (name, capname):
            mesh, col = arms[nm]
            local = mesh.copy()
            local.translate(dz=-P.YOKE_BELOW)     # pivot to the origin
            items.append((nm, local, col))
        # the servo that lives in THIS segment's cup, same frame
        for cn, cm, cc in for_part(name):
            lm = cm.copy()
            lm.translate(dz=-P.YOKE_BELOW)
            items.append((cn, lm, cc))
        seg_meta.append({"parts": [name, capname],
                         "servo": [cn for cn, _m, _c in for_part(name)],
                         "length": length})

    # ---- the loose parts -------------------------------------------------
    # Retainers, horns and the test coupon are never placed in the assembly
    # chain, so they had no 3D at all -- the parts viewer listed 13 of 22 and
    # silently dropped the rest. Laid out in a row beside the base instead,
    # in print pose, so every printed part has something to look at.
    import part_horn, part_retainers, part_screw
    loose_src = (part_horn.build() + part_retainers.build()
                 + part_screw.build() + part_keycap.build())
    loose, x = [], 0.0
    for n, m, c in loose_src:
        b = m.bounds()
        w = b[3] - b[0]
        lm = m.copy()
        lm.translate(dx=-(b[0] + b[3]) / 2 + 120.0 + x + w / 2,
                     dy=-(b[1] + b[4]) / 2 - 96.0,
                     dz=-b[2])
        items.append((n, lm, c))
        loose.append(n)
        x += w + 9.0
    rig["loose"] = loose

    shade_name, shade_mesh, shade_col = part_head.build()[0]
    sm = shade_mesh.copy()
    sm.translate(dz=-part_head.TILT)
    items.append((shade_name, sm, shade_col))

    rig["segments"] = seg_meta
    rig["shade"] = shade_name
    rig["pivot"] = [0.0, P.BSERVO_AXIS_Y, P.BJOINT_AXIS_Z]
    rig["neutral"] = {"base": -22.0, "shoulder": 58.0, "elbow": 52.0, "head": 26.0}
    # limits: how far each joint may be driven off neutral, degrees
    rig["range"] = {"base": [-38.0, 26.0], "shoulder": [-30.0, 34.0],
                    "elbow": [-34.0, 30.0], "head": [-46.0, 42.0]}

    packed = []
    for n, m, c in items:
        v, f = weld(m)
        packed.append((n, v, f, c))
    write_glb(os.path.join(WEB, "hotaru-rig.glb"), packed)
    with open(os.path.join(WEB, "rig.json"), "w") as f:
        json.dump(rig, f, indent=1)

    # ---- spec.json: what the page states as fact about the machine -------
    import audit_interfaces as AI
    man = json.load(open(os.path.join(HERE, "..", "exports", "MANIFEST.json")))
    spec = {
        "parts": [{"name": n, "mm": [round(x, 1) for x in v["bbox_mm"]],
                   "cm3": round(v["volume_mm3"] / 1000, 1),
                   "fits": v["fits_a1_mini"]}
                  for n, v in man["parts"].items()],
        "interfaces": [{"a": a, "b": b, "dof": dof, "held_by": mech}
                       for a, b, dof, mech, ok, _d in AI.ROWS],
        "stats": {
            "parts": len(man["parts"]),
            "interfaces": len(AI.ROWS),
            "joints": 4,
            "servos": 4,
            "height_mm": round(man["lamp_height_mm"]),
            "reach_mm": round(man["reach_mm"]),
            "plastic_cm3": round(sum(v["volume_mm3"] for v in man["parts"].values()) / 1000),
            "plates": 9,
            "m3": 15, "m2": 5, "printed_screws": 4,
            "bed_mm": int(man["build_plate_mm"]),
        },
    }
    # ---- one zip of every STL, and the raw pointers that serve them ------
    # Pinned to refs/heads/main, NOT to a commit: the ask was that a future
    # rebuild must not break these links, so they have to track the branch.
    import zipfile
    exp = os.path.normpath(os.path.join(HERE, "..", "exports"))
    stls = sorted(f for f in os.listdir(exp)
                  if f.endswith(".stl") and not f.startswith("plate-"))
    zpath = os.path.join(exp, "all-stls.zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for f in stls:
            z.write(os.path.join(exp, f), f)
    RAW = "https://raw.githubusercontent.com/nickthelegend/hotaru/main/exports/"
    spec["downloads"] = {
        "base": RAW,
        "zip": RAW + "all-stls.zip",
        "zip_mb": round(os.path.getsize(zpath) / 1e6, 1),
        "files": {f[:-4]: RAW + f for f in stls},
        "plates": {f[:-4]: RAW + f for f in sorted(os.listdir(exp))
                   if f.endswith(".stl") and f.startswith("plate-")},
    }

    # ---- plate detail, so the site can offer a plate at a time ----------
    # A plate is what somebody actually sends to the printer, so the page needs
    # more than a URL: what is on it, how much bed it eats, and whether the
    # slicer needs supports switched on. The supports flag comes from
    # audit_printable's own overhang measurement rather than a second copy of
    # the rule, because two copies drift and only one of them gets fixed.
    import io as _io, contextlib as _ctx
    sys.path.insert(0, HERE)
    import audit_printable as _ap
    import audit_support as _as
    import assembly as _asm
    with _ctx.redirect_stdout(_io.StringIO()):
        # Two different failure modes, so two different audits, unioned:
        #   _overhangs   face is steeper than 45 deg -- droops even though
        #                there IS material under it
        #   audit_support face has NOTHING under it -- the classifier calls
        #                that a bridge, which is only true when both ends are
        #                anchored. It is what let the shoulder's lid lugs
        #                print with their screw holes hanging in space.
        needs_support = set(_ap._overhangs())
        for _n, _m in _asm.print_items():
            _a, _w, _c = _as.unsupported(_m)
            if _a > _as.MIN_AREA:
                needs_support.add(_n)

    with open(os.path.join(exp, "PLATES.json")) as f:
        pj = json.load(f)
    plates_detail = []
    for pl_ in pj["plates"]:
        name = pl_["plate"]
        stl = os.path.join(exp, name + ".stl")
        sup = sorted(set(pl_["parts"]) & needs_support)
        plates_detail.append({
            "plate": name,
            "why": pl_["why"],
            "parts": pl_["parts"],
            "used_mm": pl_["used_mm"],
            "fits_bed": pl_["fits_bed"],
            "supports": sup,
            "mb": round(os.path.getsize(stl) / 1e6, 2) if os.path.exists(stl) else None,
            "url": RAW + name + ".stl",
        })
    # ---- the printable zip: nine plates, not twenty two loose parts ------
    # all-stls.zip is for someone who wants one specific part. Somebody who is
    # actually going to print gets the PLATES, in order, with a README that
    # says which ones need supports switched on -- because that instruction
    # lives in an audit and would otherwise never reach the person at the
    # printer.
    pz = os.path.join(exp, "plates.zip")
    readme = ["HOTARU -- printable plates", "=" * 30, "",
              f"Bed: {pj['bed_mm']:.0f} x {pj['bed_mm']:.0f} mm (Bambu A1 mini).",
              "Print in this order. Plate 1 is a 15 minute coupon that checks the",
              "printed spline fits your servo BEFORE you commit to an 8 hour tub.",
              ""]
    for i, d in enumerate(plates_detail, 1):
        x, y, z = d["used_mm"]
        readme.append(f"{i}. {d['plate']}.stl")
        readme.append(f"   {d['why']}")
        readme.append(f"   {len(d['parts'])} part(s), {x:.0f} x {y:.0f} x {z:.0f} mm")
        readme.append("   SUPPORTS: ON  (" + ", ".join(d["supports"]) + ")"
                      if d["supports"] else "   supports: not needed")
        readme.append("")
    readme += ["Supports are needed only where listed. Everything else prints",
               "as modelled; measured by cad/audit_printable.py, not asserted.",
               "", "CAD, STLs and audits: github.com/nickthelegend/hotaru"]
    with zipfile.ZipFile(pz, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        z.writestr("README.txt", "\n".join(readme))
        for d in plates_detail:
            f = d["plate"] + ".stl"
            src = os.path.join(exp, f)
            if os.path.exists(src):
                z.write(src, f)

    spec["downloads"]["plates_zip"] = RAW + "plates.zip"
    spec["downloads"]["plates_zip_mb"] = round(os.path.getsize(pz) / 1e6, 1)
    # The exact transform that takes each part from the assembled pose the
    # GLB carries into the pose it is SLICED in. Without it the parts page
    # shows a plate selection in assembly orientation, which is how somebody
    # looks at a fixed shoulder, sees its lugs pointing inward exactly as they
    # did on the failed print, and reasonably concludes nothing was fixed.
    _asm.print_items()                      # populates PRINT_SHIFT
    spec["print_pose"] = {n: {"flip": f, "dz": round(dz, 4)}
                          for n, (f, dz) in _asm.PRINT_SHIFT.items()}
    spec["plates"] = {"bed_mm": pj["bed_mm"], "list": plates_detail}

    with open(os.path.join(WEB, "spec.json"), "w") as f:
        json.dump(spec, f, indent=1)

    # ---- cache-bust the asset URLs -------------------------------------
    # Nothing here is content-hashed by a bundler, and a plain filename plus
    # any positive max-age means a redeploy never reaches a browser that
    # already has the old copy -- which is exactly what happened: two fixes
    # shipped and neither was visible. The version is a hash of the asset's
    # OWN bytes, so it changes when and only when the file does.
    import hashlib
    assets = ["site.css", "app.js", "viewer.js", "gl.js", "mark.svg"]
    vers = {}
    for a in assets:
        ap = os.path.join(WEB, a)
        if os.path.exists(ap):
            vers[a] = hashlib.sha1(open(ap, "rb").read()).hexdigest()[:8]
    for page in ("index.html", "viewer.html"):
        pp = os.path.join(WEB, page)
        if not os.path.exists(pp):
            continue
        html = open(pp).read()
        html = re.sub(r'(\./(?:' + "|".join(a.replace(".", r"\.") for a in assets)
                      + r'))(\?v=[0-9a-f]+)?', lambda m: m.group(1)
                      + ("?v=" + vers[m.group(1)[2:]] if m.group(1)[2:] in vers else ""),
                      html)
        open(pp, "w").write(html)
    # gl.js is imported from JS, so its specifier needs the same treatment
    for js in ("app.js", "viewer.js"):
        jp = os.path.join(WEB, js)
        if os.path.exists(jp) and "gl.js" in vers:
            src = open(jp).read()
            src = re.sub(r'from "\./gl\.js(\?v=[0-9a-f]+)?"',
                         'from "./gl.js?v=%s"' % vers["gl.js"], src)
            open(jp, "w").write(src)
    print("asset versions: " + ", ".join(f"{k}={v}" for k, v in vers.items()))

    tris = sum(len(f) for _n, _v, f, _c in packed)
    vts = sum(len(v) for _n, v, _f, _c in packed)
    size = os.path.getsize(os.path.join(WEB, "hotaru-rig.glb")) / 1e6
    print(f"web/hotaru-rig.glb  {len(packed)} nodes, {tris:,} tris, "
          f"{vts:,} verts, {size:.1f} MB")
    print(f"web/rig.json      {len(seg_meta)} driven segments + shade")
    print(f"web/spec.json     {spec['stats']['parts']} parts, "
          f"{spec['stats']['interfaces']} interfaces")
    print(f"exports/all-stls.zip  {len(stls)} STLs, "
          f"{spec['downloads']['zip_mb']} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())

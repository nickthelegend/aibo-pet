# AIBO — CAD

Pure-python parametric CAD. No OpenSCAD, no CadQuery, no OCC.

    python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
    .venv/bin/python cad/partlib.py        # kernel smoke test

## How it works

`partlib.py` is the kernel: 2D [shapely](https://shapely.readthedocs.io)
profiles get extruded (`prism`), lofted (`loft_solid`) or swept
(`revolve`) into closed triangle shells, which are written straight out as
binary STL / GLB.

Two rules keep it honest:

1. **No 3D CSG.** One printable part = a union of individually-watertight
   shells. Overlapping shells are fused by the slicer, not by us. 2D
   booleans (shapely) are encouraged — that is where all the real work
   happens.
2. **Overlap by `OVL` (0.2mm) where shells stack.** For an opening whose
   edges must land on exact spec numbers, use the *band-split trick*: the
   band carrying the opening stretches `OVL` into its solid neighbours,
   while each neighbour stops exactly at the opening edge — so the
   neighbour's cap face *is* the opening's edge.

`validate()` splits any mesh into connected shells and asserts each is
closed, edge-manifold, consistently wound and positive-volume. Every part
script exits non-zero if it isn't.

## Files

| File | Role |
|---|---|
| `partlib.py` | kernel: 2D profiles, `Mesh`, `prism`/`loft_solid`/`revolve`, `validate`, STL+GLB writers |
| `part_*.py` | one printable part each; runnable standalone, writes a preview GLB |
| `assembly.py` | builds every part in world position, writes print STLs + previews + `MANIFEST.json` |
| `audit_*.py` | read-only dimensional probes: component fit, print rules, SPEC compliance |

`SPEC.md` (repo root) is the dimensional contract. Numbers live there and in
the params block at the top of each part — never buried in the geometry.

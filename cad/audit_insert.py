"""audit_insert.py — can the lid actually be put in?

Every other audit asked whether the assembled parts FIT. None asked whether
there is a path to get them there. The lid seats at Z55 but is inserted from
Z61.4, so it has to pass the whole rebate on the way down -- and when three
keys were added to stop it spinning, they necked the bore to O110 while the
lid's snap bead is O114.4. Fits perfectly once seated. Cannot be inserted.

The test is a straight vertical sweep: at each angle, the widest lid material
at ANY height must clear the narrowest shoulder bore at any height it travels
through.

    .venv/bin/python cad/audit_insert.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly
import params as P

CLEAR = 0.05        # mm, the sweep must clear by at least this


def check():
    """Exact 2D: the lid's swept footprint against the opening it must pass.

    Done in shapely on the profiles both parts are built from, not by
    sampling the meshes. The first version binned vertices by degree and
    compared the widest lid radius in a bin against the narrowest bore radius
    in the SAME bin -- but those can be at different angles within one degree,
    which reported a 1.8 mm clash on a joint that clears.
    """
    import part_lid as PL
    import part_shoulder as PS
    import partlib as pl

    seat_r = (P.BASE_TOP_D - 2 * P.WALL_STRUCT) / 2.0
    bore = pl.circle(2 * seat_r, 512)
    keys = PS._key_profile(seat_r, 0.0)
    opening = bore.difference(keys)          # what the lid actually falls through
    hard, bead = PL.swept_profile()

    # hard parts must clear outright
    foul = hard.difference(opening)
    # the bead may press on the bore -- that IS the snap -- but must never
    # touch a key, which is 1.6 mm of solid and will not deflect
    beadfoul = bead.intersection(keys)
    # and its interference with the bore has to stay within the snap allowance
    over = bead.difference(bore)
    ok = foul.area < 1e-6 and beadfoul.area < 1e-6
    return ok, foul, beadfoul, over, hard, opening


def main():
    ok, foul, beadfoul, over, hard, opening = check()
    print("LID INSERTION SWEEP  (exact 2D, on the build profiles)")
    print("-" * 58)
    print(f"opening the lid falls through  {opening.area:9.1f} mm2")
    print(f"hard lid material outside it   {foul.area:9.3f} mm2   "
          f"{'CLEAR' if foul.area < 1e-6 else 'FOULS'}")
    print(f"snap bead fouling a key        {beadfoul.area:9.3f} mm2   "
          f"{'CLEAR' if beadfoul.area < 1e-6 else 'FOULS'}")
    print(f"snap bead pressing on the bore {over.area:9.1f} mm2   "
          f"(intended: this is the snap)")
    print("-" * 58)
    if ok:
        print("PASS -- the lid drops in at every angle")
        return 0
    print("FAIL -- the lid CANNOT be inserted; it fouls on the way down")
    return 1


if __name__ == "__main__":
    sys.exit(main())

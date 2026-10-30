# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""C1 bench cap: a flat 3 mm disc that screws onto the master ring's flat face (its 3 M3 inserts) and holds the 3 plunger
springs at their 8.05 mm installed length, standing in for the arm-side carrier during bench tests.  2026-10-23,
agent toolchanger-build (owner: "won't the plungers come out the back").
Run:  python toolchanger/cad/build_cap_c1.py  -> v4/stl/c1_bench_cap.stl
Screws: 3x M3 x 8 button head (owned kit) into the ring's existing inserts.  Print flat.
"""
from math import cos, sin, radians
from pathlib import Path
import sys

from build123d import Cylinder, Pos, export_stl

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
import build_changer as B  # noqa: E402

CAP_T = 3.0
M3_CLEAR = 3.4          # rule hole.p1s.pla.comp


def cap():
    c = Pos(0, 0, B.Z_RING_TOP + CAP_T / 2) * Cylinder(B.R_RING + 1.5, CAP_T)
    for a in B.RING_SCREW_ANG:
        c -= Pos(B.RING_SCREW_R * cos(radians(a)), B.RING_SCREW_R * sin(radians(a)), B.Z_RING_TOP + CAP_T / 2) * Cylinder(M3_CLEAR / 2, CAP_T + 1)
    return B.to_gripper(c)


if __name__ == "__main__":
    c = cap()
    export_stl(c, str(H / "v4" / "stl" / "c1_bench_cap.stl"), tolerance=0.005, angular_tolerance=0.05)
    bb = c.bounding_box()
    print("cap", round(c.volume / 1000, 2), "cm3, z", round(bb.min.Z, 2), round(bb.max.Z, 2))

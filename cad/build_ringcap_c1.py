# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""C1 bench RING cap: the bench cap with its middle cut out, so owner can watch the pins land in the V dowels from above.
2026-10-23, agent toolchanger-build (owner: "a version of the cap where it just is a ring so i can see the pieces
connect on the inside").

Same job as build_cap_c1.py: hold the 3 plunger springs at their 8.05 mm installed length (cap underside = ring top face,
the cage floor's place) with 3x M3x8 button heads into the ring's top-face inserts.
  inner r 25.0: the whole cavity (r 24.2: pins, ramps, V dowels) is open; the 5 mm springs (r 26.5-31.5) stay covered.
  8 mm tall, not 3: the springs push up between screws that are up to 180 deg apart (90/215/270).  As a curved beam the
  10.5 x 3 section would lift up to ~2 mm at the 315 deg plunger (14 N crest, simply supported 95 mm span) and drop its
  preload; 10.5 x 8 is ~0.2 mm, about 1 N of 10 (I goes with h^3; critic 10/23).
  Screw counterbores 6.8 (button head 5.7 + 0.5/side, rule assembly.fastener.access), 3 mm floor -> 5 mm in the insert.
Run:  python toolchanger/cad/build_ringcap_c1.py  -> v4/stl/c1_bench_ringcap.stl
Print flat, counterbores up.  Critic r1 (10/23): +0.4 insert reliefs underneath, +1 mm view chamfer; 8 mm kept (~0.2 mm lift).
"""
from math import cos, sin, radians
from pathlib import Path
import sys

from build123d import Cone, Cylinder, Pos, export_stl

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
import build_changer as B  # noqa: E402

CAP_T = 8.0
FLOOR = 3.0
R_IN = 25.0
M3_CLEAR = 3.4          # rule hole.p1s.pla.comp
CB_D = 6.8
RELIEF_D, RELIEF_H = 5.5, 0.4   # critic 10/23: clears a proud insert / melt ridge so the cap still sits flat
VIEW_CH = 1.0                   # critic 10/23: chamfer on the inner top edge widens the view down into the cavity


def ringcap():
    z0 = B.Z_RING_TOP
    c = Pos(0, 0, z0 + CAP_T / 2) * Cylinder(B.R_RING + 1.5, CAP_T)
    c -= Pos(0, 0, z0 + CAP_T / 2) * Cylinder(R_IN, CAP_T + 2)
    for a in B.RING_SCREW_ANG:
        x, y = B.RING_SCREW_R * cos(radians(a)), B.RING_SCREW_R * sin(radians(a))
        c -= Pos(x, y, z0 + CAP_T / 2) * Cylinder(M3_CLEAR / 2, CAP_T + 2)
        c -= Pos(x, y, z0 + FLOOR + (CAP_T - FLOOR) / 2 + 0.5) * Cylinder(CB_D / 2, CAP_T - FLOOR + 1)
        c -= Pos(x, y, z0 + RELIEF_H / 2 - 0.01) * Cylinder(RELIEF_D / 2, RELIEF_H + 0.02)
    c -= Pos(0, 0, z0 + CAP_T - VIEW_CH / 2 + 0.01) * Cone(R_IN, R_IN + VIEW_CH + 0.02, VIEW_CH + 0.02)
    return B.to_gripper(c)


if __name__ == "__main__":
    c = ringcap()
    export_stl(c, str(H / "v4" / "stl" / "c1_bench_ringcap.stl"), tolerance=0.005, angular_tolerance=0.05)
    bb = c.bounding_box()
    print("ringcap", round(c.volume / 1000, 2), "cm3, z", round(bb.min.Z, 2), round(bb.max.Z, 2), "valid", c.is_valid)

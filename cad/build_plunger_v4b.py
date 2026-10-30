# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Plunger v4b = plunger v4 with more running clearance.  2026-10-23, agent toolchanger-build.

Why: on the C1 print (10/23) one of the 3 plungers stopped halfway down its pocket in the ring; swapping showed the fault
travels with the plunger, not the hole (F20261023-002).  The v4 plunger had only 0.15 mm/side in both bores (shank 5.0 in
5.3, head 6.4 in 6.7), and it prints head-down, so the head carries the first-layer flare (rule print.elephant_foot).

Change (only this):
  shank 5.0 -> 4.9   (0.2 mm/side in the 5.3 guide; ball-cup rim 0.47 -> 0.42 mm, the ball is CA'd in anyway)
  head  6.4 -> 6.2   (0.25 mm/side in the 6.7 counterbore; still seats the 5 mm OD spring)
  0.4 mm x 45 deg chamfer on the head's spring face (the face on the bed when printed) against the flare.
Lengths, ball position and spring length are unchanged, so preload and protrusion are v4.

Run:  python toolchanger/cad/build_plunger_v4b.py
Writes v4/stl/master_plunger{0,1,2}_v4b.stl in their installed pose (gripper frame).
"""
import sys
from math import cos, sin, radians
from pathlib import Path

from build123d import Cylinder, Cone, Pos, Sphere, export_stl

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
import build_changer as B  # noqa: E402
import build_changer_v4 as V  # noqa: E402

SHANK_D = 4.9
HEAD_D = 6.2
CHAMFER = 0.4


def plunger_v4b():
    z_head_bot = B.Z_LIP_BOT + V.PL_GUIDE4
    z_head_top = z_head_bot + B.PL_HEAD_H
    r = HEAD_D / 2
    p = Pos(0, 0, z_head_bot + (B.PL_HEAD_H - CHAMFER) / 2) * Cylinder(r, B.PL_HEAD_H - CHAMFER)
    p += Pos(0, 0, z_head_top - CHAMFER / 2) * Cone(r, r - CHAMFER, CHAMFER)      # chamfered spring face (bed side)
    zc = B.Z_LIP_BOT - B.PL_PROTRUDE + B.PL_BALL_D / 2
    p += Pos(0, 0, (z_head_bot + zc) / 2) * Cylinder(SHANK_D / 2, z_head_bot - zc + 0.01)
    p -= Pos(0, 0, zc) * Sphere(B.PL_BALL_D / 2 + 0.05)
    return p


if __name__ == "__main__":
    for k, a in enumerate(V.PL_ANG4):
        part = B.to_gripper(Pos(B.R_PL * cos(radians(a)), B.R_PL * sin(radians(a)), 0) * plunger_v4b())
        out = H / "v4" / "stl" / f"master_plunger{k}_v4b.stl"
        export_stl(part, str(out), tolerance=0.005, angular_tolerance=0.05)
        bb = part.bounding_box()
        print(out.name, round(part.volume, 1), "mm3, z", round(bb.min.Z, 2), round(bb.max.Z, 2), "valid", part.is_valid)

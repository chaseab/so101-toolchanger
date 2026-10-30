# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Ring v4q = ring v4p with a screw + heat-set insert BESIDE EACH SPRING.  2026-10-24, agent toolchanger-master.

Why (owner 10/24, bench): the 3 mm bench cap visibly bowed where a spring sat far from a screw.  v4p inserts are at r 31,
90/215/270 deg; the plungers are at r 29, 75/195/315, so the 315 spring was 45 deg (~23 mm) from the nearest screw.
Change (only this): the 3 top-face M3 insert bores move to RING_SCREWS_V4Q = r 30 at 90 / 180 / 300 deg, each 15 deg from
its plunger (75 / 195 / 315): centre distance to the plunger counterbore 7.8 mm -> 2.4 mm PLA wall (rule
insert.heatset.wall_min 1.6).  r 30 instead of 31 gives a 2.0 mm outer wall (v4p had 1.0) and 3.8 mm to the cavity.
Bores: 4.0 x 6.7 deep (rule: insert L 5.7 + 1) + 3.4 x 2.0 screw-tip clearance; 0.4 x 0.4 mouth chamfer (flush ring).
The old 215 / 270 bores are dropped (the matching cage floor keeps holes for them, so the bench ring v4p still bolts on).
Everything else = ring v4p, including the dowel drop-in slots and the print pause at print z 6.0 -> 6.2.

Run:  python toolchanger/cad/build_ring_v4q.py
Writes v4/stl/master_ring_v4q.stl, v4/gate_parts/ring_v4q_at_pause.stl, v4/ring_v4q.json
"""
import json
import sys
from math import cos, sin, radians, hypot
from pathlib import Path

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))

RING_SCREWS_V4Q = ((30.0, 90.0), (30.0, 180.0), (30.0, 300.0))   # (r, deg) in the master frame (before to_gripper)
INS_D, INS_DEPTH, TIP_D, TIP_H, MOUTH = 4.0, 6.7, 3.4, 2.0, 0.4


def ring_v4q():
    from build123d import Cylinder, Pos
    import build_changer as B
    import build_ring_v4p as RP
    saved = B.RING_SCREW_ANG
    B.RING_SCREW_ANG = ()                       # ring_v4_clean() cuts no insert bores
    try:
        b = RP.ring_v4_clean()
    finally:
        B.RING_SCREW_ANG = saved
    for k in range(3):
        b -= RP.slot(120 * k)
    zt = B.Z_RING_TOP
    for r, a in RING_SCREWS_V4Q:
        x, y = r * cos(radians(a)), r * sin(radians(a))
        b -= Pos(x, y, zt - INS_DEPTH / 2 + 0.5) * Cylinder(INS_D / 2, INS_DEPTH + 1)
        b -= Pos(x, y, zt - INS_DEPTH - TIP_H / 2 + 0.01) * Cylinder(TIP_D / 2, TIP_H + 0.02)
        b -= Pos(x, y, zt - MOUTH / 2 + 0.25) * Cylinder(INS_D / 2 + MOUTH, MOUTH + 0.5)
    return b


def checks():
    import build_changer as B
    import build_changer_v4 as V
    out = {}
    for r, a in RING_SCREWS_V4Q:
        x, y = r * cos(radians(a)), r * sin(radians(a))
        dpl = min(((hypot(x - B.R_PL * cos(radians(p)), y - B.R_PL * sin(radians(p))), p) for p in V.PL_ANG4))
        out[f"{a:g}"] = {"nearest_plunger_deg": dpl[1], "centre_dist": round(dpl[0], 2),
                         "wall_to_plunger_bore": round(dpl[0] - INS_D / 2 - (B.PL_HEAD_D / 2 + 0.15), 2),
                         "wall_outer": round(B.R_RING - r - INS_D / 2, 2), "wall_cavity": round(r - INS_D / 2 - B.R_CAV, 2),
                         "deg_to_plunger": round(min(abs((a - p + 180) % 360 - 180) for p in V.PL_ANG4), 1)}
    return out


def main():
    from build123d import Box, Pos, export_stl
    import build_changer as B
    import build_ring_v4p as RP
    ring = B.to_gripper(ring_v4q())
    export_stl(ring, str(H / "v4" / "stl" / "master_ring_v4q.stl"), tolerance=0.005, angular_tolerance=0.05)
    cut = ring & (Pos(0, 0, B.Z_LIP_BOT + RP.PAUSE_PRINT_Z / 2) * Box(200, 200, RP.PAUSE_PRINT_Z))
    export_stl(cut, str(H / "v4" / "gate_parts" / "ring_v4q_at_pause.stl"), tolerance=0.005, angular_tolerance=0.05)
    bb = ring.bounding_box()
    info = {"part": "master_ring_v4q", "cm3": round(ring.volume / 1000, 2), "valid": ring.is_valid,
            "bbox": [round(v, 2) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)],
            "RING_SCREWS_V4Q": RING_SCREWS_V4Q, "PAUSE_PRINT_Z": round(RP.PAUSE_PRINT_Z, 3), "insert_checks": checks()}
    (H / "v4" / "ring_v4q.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()

# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Ring v4p = ring v4 with DROP-IN dowel slots for a print pause.  2026-10-23, agent toolchanger-build.

Why: the C1 print (10/23) took no dowels.  The v4 grooves are 3.05 mm horizontal bores, which print about 2.85 (rule
fit.peg.pla, finding F20261023-001).  owner: seat the 6 dowels during the print instead.

Change (only this): each V's two 3.05 bores become one U slot = two 3.2 half-rounds (each dowel centres in its own
round bottom; 3.3 = rule 3.2 + 0.1 because the pair shares one slot) + straight walls up to print z 6.0 (gripper z -55.0), 0.3 mm above the dowel tops.  The print pauses
before the 6.0 -> 6.2 layer; owner drops 6 dowels in; the next layers bridge over the outer ~3 mm of each slot (r > 24.75,
where the V-trough relief ends) and the 0.8 mm lip wall stops the inner end.  That keeps them in the ring, but they
can tilt ~0.1 rad in handling; in service the pins' preload presses them onto the slot floor (critic 10/23).
Keep the pause short (nozzle stays hot).
Their upper faces stay open in the V trough, as before.  Every other dimension is ring v4.

Run:  python toolchanger/cad/build_ring_v4p.py
Writes toolchanger/cad/v4/stl/master_ring_v4p.stl
"""
import json
import sys
from math import cos, sin, radians, tan
from pathlib import Path

from build123d import Cylinder, Pos, Rot, Box, export_stl

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
import build_changer as B  # noqa: E402
import build_changer_v4 as V  # noqa: E402

SLOT_D = 3.3                     # rule fit.peg.pla says 3.2; +0.1 because two dowels share one slot (critic 10/23: 6.2 could print ~6.0)
Z_SLOT_TOP = -55.0               # gripper z; ring lip bottom (-61) is on the bed, so print z 6.0 = a 0.2 layer boundary
PAUSE_PRINT_Z = Z_SLOT_TOP - B.Z_LIP_BOT   # 6.0: pause before the layer that tops out at 6.2


def slot(a0):
    zd = B.Z_LIP_TOP + B.H_DOWEL
    r0, r1 = V.DOWEL_RIN4 - 0.2, V.DOWEL_RIN4 + B.DOWEL_L + 0.2
    s = None
    for t in (-1, 1):
        c = B.radial_rod(SLOT_D, r0, r1, a0 + B.PHI_V, t * B.DOWEL_D / 2, zd)
        s = c if s is None else s + c
    w = B.DOWEL_D + SLOT_D           # both dowels side by side: centres +-1.5, each 3.2 wide
    zb = zd - 0.3                   # start the walls 0.3 below the dowel centres: no faces coincident with the half-rounds
    h = Z_SLOT_TOP - zb
    s += Rot(0, 0, a0 + B.PHI_V) * Pos((r0 + r1) / 2, 0, zb + h / 2) * Box(r1 - r0, w, h)
    return s


def ring_v4_clean():
    """Copy of build_changer_v4.ring_v4 with ONE change: the ramp / crest / stop-block sectors run to R_CAV + 0.3 (into the
    solid wall) instead of ending exactly on the cavity wall.  Same solid (the wall is already there); it removes the
    coincident faces that made every v4 ring STL non-manifold at r 24.2 (gate section fill broke on it, 10/23)."""
    zt, zb = B.Z_RING_TOP, B.Z_LIP_BOT
    b = Pos(0, 0, (zt + zb) / 2) * Cylinder(B.R_RING, zt - zb)
    b += Pos(0, 0, (zt + zb + 3.0) / 2) * Cylinder(B.R_RING + 1.5, zt - zb - 3.0)
    b -= Pos(0, 0, (zt + B.Z_LIP_TOP) / 2 + 0.01) * Cylinder(B.R_CAV, zt - B.Z_LIP_TOP + 0.02)
    b -= Pos(0, 0, (B.Z_LIP_TOP + zb) / 2) * Cylinder(V.R_BORE4, B.LIP_T + 2)
    from build123d import Cone
    b -= Pos(0, 0, zb + B.BORE_CHAMFER / 2) * Cone(V.R_BORE4 + B.BORE_CHAMFER, V.R_BORE4, B.BORE_CHAMFER)
    zd = B.Z_LIP_TOP + B.H_DOWEL
    for k in range(3):
        a0 = 120 * k
        b -= B.sector(V.R_BORE4 - 1, V.R_WIN4, a0 - B.WIN, a0 + B.WIN, zb - 1, B.Z_LIP_TOP + 0.01)
        b -= B.sector(V.R_BORE4 - 1, V.R_WIN4 + 1.7, a0 - B.WIN - 6, a0 + B.WIN + 6, zb - 0.01, zb + 1.0)
        steps = int(B.RAMP[1] - B.RAMP[0])
        for i in range(steps):
            h = B.H_CREST * (i + 1) / steps
            b += B.sector(V.RELIEF_RIN, B.R_CAV + 0.3, a0 + B.RAMP[0] + i, a0 + B.RAMP[0] + i + 1.02, B.Z_LIP_TOP - 0.01, B.Z_LIP_TOP + h)
        b += B.sector(V.RELIEF_RIN, B.R_CAV + 0.3, a0 + B.RAMP[1], a0 + B.PHI_STOP[0], B.Z_LIP_TOP - 0.01, B.Z_LIP_TOP + B.H_CREST)
        b += B.sector(V.RELIEF_RIN, B.R_CAV + 0.3, a0 + B.PHI_STOP[0], a0 + B.PHI_STOP[1], B.Z_LIP_TOP - 0.01, B.Z_LIP_TOP + B.H_STOP)
        top = B.H_CREST + 0.8
        hw = (top - 0.3) / tan(radians(30))
        b -= Pos(0, 0, B.Z_LIP_TOP) * B.radial_prism([(-hw, top), (hw, top), (0, 0.3)], V.RELIEF_RIN - 0.3, B.R_CAV + 0.5, a0 + B.PHI_V)  # R1: no sliver at the bore
        stop = B.radial_prism([(B.STOP_T[0], -0.01), (B.STOP_T[1], -0.01), (B.STOP_T[1], B.H_STOP), (B.STOP_T[0], B.H_STOP)],
                              V.RELIEF_RIN, B.R_CAV + 0.3, a0 + B.PHI_V)
        b += Pos(0, 0, B.Z_LIP_TOP) * stop
        for s in (-1, 1):
            b -= B.radial_rod(B.DOWEL_D + 0.05, V.DOWEL_RIN4 - 0.2, V.DOWEL_RIN4 + B.DOWEL_L + 0.2, a0 + B.PHI_V, s * B.DOWEL_D / 2, zd)
    for a in V.PL_ANG4:
        x, y = B.R_PL * cos(radians(a)), B.R_PL * sin(radians(a))
        b -= Pos(x, y, zb + V.PL_GUIDE4 / 2 - 0.01) * Cylinder(B.PL_SHANK_D / 2 + 0.15, V.PL_GUIDE4 + 0.02)
        b -= Pos(x, y, (zt + zb + V.PL_GUIDE4) / 2 + 0.01) * Cylinder(B.PL_HEAD_D / 2 + 0.15, zt - zb - V.PL_GUIDE4 + 0.02)
    for a in B.RING_SCREW_ANG:
        x, y = B.RING_SCREW_R * cos(radians(a)), B.RING_SCREW_R * sin(radians(a))
        b -= Pos(x, y, zt - 2.85) * Cylinder(2.0, 5.7)
        b -= Pos(x, y, zt - 5.7 - 2.0) * Cylinder(1.3, 4.0)
    return b


def ring_v4p():
    """ring_v4 with its six 3.05 dowel bores replaced by three drop-in U slots."""
    b = ring_v4_clean()
    for k in range(3):
        b -= slot(120 * k)
    return b


def main():
    ring = B.to_gripper(ring_v4p())
    out = H / "v4" / "stl" / "master_ring_v4p.stl"
    export_stl(ring, str(out), tolerance=0.005, angular_tolerance=0.05)
    from build123d import Box as _Bx, Pos as _P
    cut = ring & (_P(0, 0, B.Z_LIP_BOT + PAUSE_PRINT_Z / 2) * _Bx(200, 200, PAUSE_PRINT_Z))   # the ring as it stands at the pause
    export_stl(cut, str(H / "v4" / "gate_parts" / "ring_v4p_at_pause.stl"), tolerance=0.005, angular_tolerance=0.05)
    bb = ring.bounding_box()
    info = {"part": "master_ring_v4p", "cm3": round(ring.volume / 1000, 2),
            "bbox": [round(v, 2) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)],
            "SLOT_D": SLOT_D, "Z_SLOT_TOP_gripper": Z_SLOT_TOP, "PAUSE_PRINT_Z": round(PAUSE_PRINT_Z, 3),
            "dowel_top_print_z": round(B.Z_LIP_TOP + B.H_DOWEL + B.DOWEL_D / 2 - B.Z_LIP_BOT, 3)}
    (H / "v4" / "ring_v4p.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()

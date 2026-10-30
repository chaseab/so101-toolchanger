# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Dock table mount v1: a frame that holds the C0 dock post (dock_post_v4, printed 10/25) fixed on the table.  2026-10-25,
agent toolchanger-build.  Blocker (c): the dock must sit ~310 mm from the pan axis and move < 0.5 mm (TEST_PLAN D2).

Why a frame and not tape alone: the 45-deg lock twist puts up to ~2.3 N.m (DESIGN_v4, worst corner) on the dock about the
tool axis, 100 mm above the table.  The dock's 124 mm foot would need ~3.8 kg of weight to resist that on its own, and its
foot has no holes (no drilling: no-hand-fab).
  FRAME  9 mm ring around the foot (cutout = foot + 0.25/side), taped (VHB / carpet tape) or screwed to the table through 4
         countersunk corner holes (M4 / #6 wood screws).  The foot sits on the TABLE, so the dock height stays 100 mm.
  BARS   2 clamp bars across the foot's free strips (arm side x -18..-3, back x 58.5..69 in the dock frame), each with a
         presser over the foot that stands 0.6 mm proud: 2x M3x10 countersunk per bar into heat-set inserts in the frame.
  MARKS  V-notches at y = 0 on the front and back edges (the radial line to the pan axis) and a V on both side faces at
         dock x = -7 = the seated coupling face: tape-measure 310 mm from the pan axis to those marks.
DOCK frame (build_changer): origin on the wall's arm-side face at the tool axis, +x away from the arm, +z up; table z = -100.
Run: python toolchanger/cad/build_dock_base.py  -> v4/stl/db_*.stl, v4/dock_base.json
"""
import json
import sys
from pathlib import Path

from build123d import Box, Cylinder, Cone, Pos, Rot, export_stl, Polygon, extrude

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
import build_changer as B  # noqa: E402

OUT = H / "v4" / "stl"
ZT = -B.DOCK_H                          # table
FOOT_X, FOOT_HY, FOOT_T = (-20.0, 70.0), B.WALL_HW, 6.0      # dock_post_v4 foot: Box(90, 124, 6) at x 25
CLR = 0.25                              # r2: the foot is unmeasured and its first layer flares; VHB under the foot takes up the play
FR_X, FR_HY, FR_T = (-35.0, 85.0), 80.0, 9.0          # r1: 9 thick -> 2.3 mm floor under the insert bores
INS_D, INS_L = 4.0, 6.7                 # rule insert.heatset.M3.bore
BARS = {"front": (-18.0, -3.0), "back": (58.5, 69.0)}
BAR_T, PRESS = 6.0, 0.6                 # bar thickness; presser stands 0.6 proud of the frame-top seat (r1: 0.3 gave 4-11 N)
SCREW_Y = 71.0
CSK_SINK = 1.2                          # head sunk 1.2 below the bar top: M3x10 keeps ~3.2-3.8 mm in the insert with the 0.6 presser
WOOD = [(-28.0, -73.0), (-28.0, 73.0), (78.0, -73.0), (78.0, 73.0)]   # corner screw holes (M4 / #6)
COUPLING_FACE_X = -7.0                  # dock x of the seated coupling face (plate back 2 mm off the wall + 5 mm plate)


def frame():
    x0, x1 = FR_X
    f = Pos((x0 + x1) / 2, 0, ZT + FR_T / 2) * Box(x1 - x0, 2 * FR_HY, FR_T)
    fx0, fx1 = FOOT_X[0] - CLR, FOOT_X[1] + CLR
    f -= Pos((fx0 + fx1) / 2, 0, ZT + FR_T / 2) * Box(fx1 - fx0, 2 * (FOOT_HY + CLR), FR_T + 2)
    for xa, xb in BARS.values():
        for sy in (1, -1):
            f -= Pos((xa + xb) / 2, sy * SCREW_Y, ZT + FR_T - INS_L / 2 + 0.01) * Cylinder(INS_D / 2, INS_L)
    for x, y in WOOD:                    # 4.5 clearance, 90-deg countersink 9.0 from the top
        f -= Pos(x, y, ZT + FR_T / 2) * Cylinder(2.25, FR_T + 2)
        f -= Pos(x, y, ZT + FR_T - 2.25 + 0.01) * Cone(2.25, 4.5, 2.25)
    for x in (x0, x1):                   # V-notches on the radial line
        f -= Pos(x, 0, ZT + FR_T / 2) * Rot(0, 0, 45) * Box(4.0, 4.0, FR_T + 2)
    for sy in (1, -1):                   # coupling-face mark: a vertical V on each side face (r1: a top groove hid under the bar)
        f -= Pos(COUPLING_FACE_X, sy * FR_HY, ZT + FR_T / 2) * Rot(0, 0, 45) * Box(2.0, 2.0, FR_T + 2)
    return f


def bar(xa, xb):
    """Flat bar on the frame top (0.6 mm gap there before the screws pull it down) + presser on the foot top."""
    w = xb - xa
    z_seat = ZT + FR_T + PRESS            # bar underside over the frame when the presser just touches the foot
    b = Pos((xa + xb) / 2, 0, z_seat + BAR_T / 2) * Box(w, 2 * (SCREW_Y + 6.0), BAR_T)
    zf = ZT + FOOT_T
    b += Pos((xa + xb) / 2, 0, (zf + z_seat) / 2 + 0.005) * Box(w, 2 * (FOOT_HY - 1.0), z_seat - zf + 0.01)
    for sy in (1, -1):
        b -= Pos((xa + xb) / 2, sy * SCREW_Y, z_seat + BAR_T / 2) * Cylinder(1.7, BAR_T + 2)
        b -= Pos((xa + xb) / 2, sy * SCREW_Y, z_seat + BAR_T - 1.65 - CSK_SINK + 0.01) * Cone(1.7, 3.35, 1.65)   # M3 csk, sunk
        b -= Pos((xa + xb) / 2, sy * SCREW_Y, z_seat + BAR_T - CSK_SINK / 2 + 0.01) * Cylinder(3.4, CSK_SINK + 0.02)
    return b


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    parts = {"db_frame": frame(), "db_bar_front": bar(*BARS["front"]), "db_bar_back": bar(*BARS["back"])}
    info = {"parts": {}, "coupling_face_dock_x": COUPLING_FACE_X,
            "place": "tape-measure 310 mm from the pan axis to the side-face V marks; both V-notches on the line to the pan axis"}
    for n, s in parts.items():
        export_stl(s, str(OUT / f"{n}.stl"), tolerance=0.01, angular_tolerance=0.1)
        bb = s.bounding_box()
        info["parts"][n] = {"cm3": round(s.volume / 1000, 2), "valid": s.is_valid,
                            "bbox": [round(v, 2) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)]}
    import build_changer_v4 as V
    d = V.dock_post_v4()
    for n, s in parts.items():
        info["parts"][n]["vs_dock_post_mm3"] = round(float((s & d).volume), 3)
    (H / "v4" / "dock_base.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()

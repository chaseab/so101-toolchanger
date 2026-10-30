# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""FR1: TPU fin ray parallel gripper for the SO-101 v4 tool changer.  2026-10-28, agent tc-tools2.
Run:  python toolchanger/tools_x2/build_fr1.py
Writes tools_x2/fr1/stl/*.stl (assembled pose at s = 0 = closed, TOOL frame) + fr1/parts.json.

What it is: GT1's drive, unchanged inside (blade -> rotor_v4 -> gt_shaft -> m1.25 z16 pinion -> 2 racks, solid stops at
s = 0 / 21 mm per jaw = 0 / 120.3 deg), in a re-styled housing, with two TPU 95A fin ray fingers instead of rigid pads.
  * body / cover = GT1's solids intersected with a filleted envelope (R7 corners, R3 top edges, rounded skirt ends), so every
    internal feature (tunnel, grooves, stop posts, bores, insert pockets, magnet pocket, screw pilots) is GT1's.
  * jaws = GT1's jaw above z -47 (rack, tongue, keel, webs between the cover skirts: same sliding geometry) + a new
    "shoe" that clamps the fin ray's root.  The shoe's mouth is cut square to the fin ray's back flank (the clamp face of
    the validated v3 profile), so the shoe's underside is slanted 24 deg: it looks like what it does.
  * fin ray = gripper/tools/finray_geom.py v3 profile (H24, L50, 0.8 ribs at -55 deg, fillet 0.3; arbiter-validated
    2026-09-25), 18 wide, root block 10 (was 6) so the pocket is deep enough to clamp, mounted with the FRONT flank parallel
    to the tool axis (parallel jaws: the flanks stay parallel through the stroke, tips meet for thin wires).
TPU attachment, no glue: the root block pushes up into the shoe pocket (3 walls + 2 cheeks, open toward the other jaw),
  one M3x25 button head across the cheeks THROUGH the TPU (3.4 hole) into an M3 nut in a hex trap.  Snug only.
Grip: FORCE-limited (critic r1): run FR1 sessions with tool_session.apply_session_limits(bus, 0.05) -> <= 2.5 N per jaw, so a
  full-close command can't crush the fin rays (v3 FE: 1.2-2.4 N at 5 mm, x 18/16).  Closed stop: the fin rays preload each other
  2 mm (thin wires get >= 1.75 mm of squeeze per finger).  Open stop: 40 mm between the flanks.
"""
import json
import sys
from math import atan2, cos, sin, degrees, radians, sqrt
from pathlib import Path

from build123d import (Box, Cylinder, Plane, Polygon, Pos, Rot, Axis, export_stl, extrude, fillet, chamfer, Text, Location)

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
from common import B, V, GT, bx, rbox, coupling, coupling_hardware, engrave_py, insert_leads, neck_insert_mouths  # noqa: E402

sys.path.insert(0, str(H.parents[1] / "gripper" / "tools"))
import finray_geom as FG  # noqa: E402

OUT = H / "fr1" / "stl"
# ---------------------------------------------------------------- fin ray (v3 profile, wider + deeper root)
FR = dict(FG.DESIGN)
# tpu-iter 10/30 (v2): back flank 2.0 -> 1.6, tip 1.5 -> 1.2 (beam sim V9: +5 % wrap (within sim noise), same force; see tpu_iter/TPU_ITER_1030.html); v1 kept in fr1_v1/
FR.update(width=18.0, root_block=10.0, t_back=1.6, h_tip=1.2)
L, HH, HT = FR["L"], FR["H"], FR["h_tip"]
ALPHA = atan2(HH - HT, L)                    # 24.2 deg: front flank vs back flank
SA, CA = sin(ALPHA), cos(ALPHA)
X_FRONT = -1.0                               # front flank SURFACE at s = 0: 1 mm past the centre, so at the closed stop the two
                                             # fin rays preload each other 2 mm (critic r1: a 1.5 mm wire got only 0.6 mm of squeeze)
SHOE_X_IN = 0.2                              # the rigid shoes stop at x 0.2 (only the TPU crosses the centre)
XF = X_FRONT + FR["t_front"] / 2             # front flank centreline
W2 = FR["width"] / 2
CLR = 0.15                                   # TPU-to-pocket clearance (sides, outer wall)
CHEEK = 3.5                                  # r1: 3.5 so the one-sided nut trap keeps a 0.9 floor toward the pocket
WALL = 3.0
ROOF = 3.0
ZA = -47.0                                   # shoe's top inner corner sits on GT1's web bottom


def uv2xz(u, v, zfr):
    """profile (u along the back flank from the clamp, v height; front flank at v = H) -> tool (x, z), jaw 1, s = 0."""
    return (XF - SA * u - CA * (v - HH), zfr - CA * u + SA * (v - HH))


# shoe corner A = (u -ROOT-ROOF, v H + t_front/2) sits at (~5.5, ZA): solve the root height zfr
_UA, _VA = -FR["root_block"] - ROOF, HH + FR["t_front"] / 2
ZFR = ZA - (uv2xz(_UA, _VA, 0.0)[1])
BOLT_UV = (-FR["root_block"] / 2, HH / 2)


def xz(u, v):
    return uv2xz(u, v, ZFR)


def ext_y(pts, y0, y1):
    """(x, z) polygon extruded over y0..y1 (winding-proof: extrude, then move the result onto y0..y1)."""
    s = extrude(Plane.XZ * Polygon(*pts, align=None), amount=y1 - y0)
    return Pos(0, y0 - s.bounding_box().min.Y, 0) * s


def finray():
    """TPU fin ray (jaw 1, assembled): the v3 profile mapped into the tool x-z plane, extruded +-W2 in y, 3.4 bolt hole."""
    g = FG.profile(FR)
    ext = [xz(u, v) for u, v in list(g.exterior.coords)[:-1]]
    f = ext_y(ext, -W2, W2)
    for hole in g.interiors:
        pts = [xz(u, v) for u, v in list(hole.coords)[:-1]]
        f -= ext_y(pts, -W2 - 1, W2 + 1)
    bx_, bz = xz(*BOLT_UV)
    f -= Pos(bx_, 0, bz) * Rot(90, 0, 0) * Cylinder(FR["bolt_d"] / 2, 2 * W2 + 2)
    return f


def _uv_prism(pts_uv, y0, y1):
    return ext_y([xz(u, v) for u, v in pts_uv], y0, y1)


def shoe():
    """Root clamp under GT1's web: rotated block (u -13..0, v -t_back/2-CLR-WALL .. front), pocket u -10..0 open at the
    mouth (u = 0) and toward the other jaw (v > H), cheeks at +-(W2+CLR .. +CHEEK), M3 across with a nut trap on -y."""
    rb, tb, tf = FR["root_block"], FR["t_back"], FR["t_front"]
    v_out = -tb / 2 - CLR - WALL
    v_in = HH + tf / 2
    yo = W2 + CLR + CHEEK
    s = _uv_prism([(-rb - ROOF, v_out), (0, v_out), (0, v_in), (-rb - ROOF, v_in)], -yo, yo)
    # gusset to the web: inner face continues the web's x 5 face, outer face 45 deg down to the shoe's outer top corner
    (xa, za), (xb, zb) = xz(-rb - ROOF, v_in), xz(-rb - ROOF, v_out)
    z45 = zb + (xb - GT.WEB["x"][1])                      # where a 45-deg line from B meets the web's outer face
    gus = [(GT.WEB["x"][0], ZA + 0.5), (GT.WEB["x"][1], ZA + 0.5), (GT.WEB["x"][1], z45), (xb, zb), (xa, za)]
    s += ext_y(gus, -yo, yo)
    # soften: edges along y on the outer side
    (xc, zc), (xd, zd) = xz(0, v_out), xz(0, v_in)
    for (px, pz), r in (((xb, zb), 6.0), ((xc, zc), 2.0), ((GT.WEB["x"][1], z45), 4.0), ((xd, zd), 0.6)):
        try:                                              # cosmetic: keep the solid if OCC refuses one
            e = s.edges().filter_by(Axis.Y).filter_by(lambda e: abs(e.center().X - px) < 0.3 and abs(e.center().Z - pz) < 0.3)
            s = fillet(e, r)
        except Exception as ex:
            print("shoe fillet skipped at", round(px, 1), round(pz, 1), ex)
    try:
        s = fillet(s.edges().filter_by(Axis.Y, reverse=True).filter_by(lambda e: abs(abs(e.center().Y) - yo) < 0.01), 0.8)
    except Exception as ex:
        print("cheek edge fillet skipped", ex)
    # pocket
    s -= _uv_prism([(-rb, -tb / 2 - CLR), (1.0, -tb / 2 - CLR), (1.0, v_in + 5), (-rb, v_in + 5)], -(W2 + CLR), W2 + CLR)
    # bolt (along y): 3.4 clearance, head on +y (button, sits on the cheek), nut trap 3.0 deep on -y
    bx_, bz = xz(*BOLT_UV)
    s -= Pos(bx_, 0, bz) * Rot(90, 0, 0) * Cylinder(1.7, 2 * yo + 2)
    nut = extrude(Polygon(*GT.hexagon(5.7), align=None), amount=2.6)          # r1 (both critics): ONE-sided 2.6 deep trap
    nut = Pos(0, -yo - nut.bounding_box().min.Z, 0) * Pos(bx_, 0, bz) * Rot(-90, 0, 0) * nut
    nb = nut.bounding_box()
    assert abs(nb.min.Y + yo) < 0.05 and abs(nb.max.Y - (-yo + 2.6)) < 0.05, (nb.min.Y, nb.max.Y)
    s -= nut
    s -= bx((-30.0, SHOE_X_IN), (-20, 20), (-80.0, -40.0))   # rigid shoe stays at x >= 0.2; the TPU front crosses the centre
    return s


def jaw1():
    """GT1's jaw 1 above z -47 (rack, back tongue, keel, webs), unchanged, + the fin ray shoe."""
    j = GT.jaw1() & bx((-40, 40), (-20, 20), (-20.0, ZA))
    return j + shoe()


# ---------------------------------------------------------------- housing (GT1's solids, filleted envelope)
R_CORNER, R_TOP = 7.0, 3.0


def body():
    env = rbox((-GT.BX, GT.BX), (-GT.BY, GT.BY), (-21.0, GT.Z_BOT), r_v=R_CORNER)
    try:
        env = fillet(env.edges().group_by(Axis.Z)[-1], R_TOP)      # top perimeter (the neck sits inside it)
    except Exception as e:
        print("top fillet skipped:", e)
    env += bx((-20, 20), (-20, 20), (-4.0, -21.5))                 # neck region untouched
    b = insert_leads(GT.body() & env, neck_insert_mouths())
    # engraved tool ID on the +y face (0.4 deep): the swap camera / owner can tell the tools apart at a glance
    b = engrave_py(b, "FR1", 24.0, (-21.0 + GT.Z_BOT) / 2 - 0.5, GT.BY, size=6.0)
    return b


def cover():
    env = rbox((-GT.BX, GT.BX), (-GT.BY, GT.BY), (GT.Z_BOT, GT.COVER["z_skirt"]), r_v=R_CORNER)
    ends = rbox((-GT.BX, GT.BX), (-GT.BY - 1, GT.BY + 1), (GT.Z_BOT + 8, GT.COVER["z_skirt"]), r_v=0.0)
    ends = fillet(ends.edges().filter_by(Axis.Y).group_by(Axis.Z)[0], 6.0)    # rounded skirt ends at the bottom
    env = env & (ends + bx((-GT.BX, GT.BX), (-GT.BY - 1, GT.BY + 1), (GT.Z_BOT + 0.01, GT.Z_BOT - 6)))
    return GT.cover() & env


# ---------------------------------------------------------------- hardware (tool frame, jaw poses at s = 0)
def hardware():
    hw = coupling_hardware()
    hw.append(("shaft_screw", "M3x8 button + M3 washer", (0.0, 0.0, GT.FACE[1]), (0, 0, -1), 8.0, "bolt"))
    for i, (x, y) in enumerate(GT.COVER["screw"]):
        hw.append((f"cover_screw{i + 1}", "M3x8 button (self-taps)", (x, y, GT.COVER["z_plate"] + 1.0), (0, 0, -1), 8.0, "bolt"))   # gate r4: M3x10 bottomed in the pilot
    bx_, bz = xz(*BOLT_UV)
    yo = W2 + CLR + CHEEK
    for side, name in ((1, "j1"), (-1, "j2")):
        p = (bx_ * side, yo * side, bz)
        hw.append((f"finger_bolt_{name}", "M3x25 button", p, (0, side, 0), 25.0, "bolt"))
        hw.append((f"finger_nut_{name}", "M3 nut", (bx_ * side, -(yo - 1.5) * side, bz), (0, -side, 0), 2.4, "nut"))   # nut in the -y trap
    return hw


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    j1 = jaw1()
    fr = finray()
    parts = {**coupling(), "body": body(), "cover": cover(), "pinion": GT.pinion(), "jaw1": j1, "jaw2": Rot(0, 0, 180) * j1,
             "finray1": fr, "finray2": Rot(0, 0, 180) * fr}
    info = {"parts": {}, "zfr": ZFR, "alpha_deg": degrees(ALPHA), "bolt_xz": xz(*BOLT_UV),
            "tip_z": min(xz(L + 1, 0)[1], xz(L + 1, HT)[1]),
            "moving": {"jaw1": "rack1", "finray1": "rack1", "jaw2": "rack2", "finray2": "rack2", "pinion": "pinion",
                       "shaft": "pinion", "rotor": "pinion"},
            "hardware": hardware()}
    for name, s in parts.items():
        export_stl(s, str(OUT / f"{name}.stl"), tolerance=0.01, angular_tolerance=0.1)
        bb = s.bounding_box()
        info["parts"][name] = {"cm3": round(s.volume / 1000, 3), "valid": s.is_valid,
                               "bbox": [round(v, 2) for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)]}
        print(name, info["parts"][name])
    # open-stop pose (s = 21 per jaw) for the gate sidecar / pictures: the fin rays stand 40 mm apart
    so = GT.S_OPEN
    for n, sh in (("jaw1", Pos(so, 0, 0) * j1), ("jaw2", Rot(0, 0, 180) * Pos(so, 0, 0) * j1),
                  ("finray1", Pos(so, 0, 0) * fr), ("finray2", Rot(0, 0, 180) * Pos(so, 0, 0) * fr)):
        export_stl(sh, str(OUT / f"{n}_open.stl"), tolerance=0.01, angular_tolerance=0.1)
    (H / "fr1" / "parts.json").write_text(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()

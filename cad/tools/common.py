# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Shared bits for the round-2 tools (tools_x2).  2026-10-28, agent tc-tools2.

TOOL frame = build_gripper_tool / build_changer: coupling face z = 0, tool along -z, +x = UP in the dock (the tool's -x face
rests on the dock's tail rib at tool z -25..-41, x = -39.3).

Every tool keeps GT1's coupling half EXACTLY: gt_plate (+3 steel 3x12 pins, 4 countersunk M3x10 into the neck inserts at
r 19), rotor_v4, gt_shaft and GT1's neck (36 sq, 1.5 chamfers, rotor seat, shaft bearing, 4 M3 inserts, flush N52 6x3
neck magnet on the -x face).  The neck is cut straight out of GT.body(), so it is the same solid as the gated GT1 print.
"""
import sys
from math import cos, sin, radians
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot, Axis, fillet, chamfer

H = Path(__file__).resolve().parent
CAD = H.parent / "cad"
sys.path.insert(0, str(CAD))
sys.path.insert(0, str(H.parent / "tools_x"))
import build_changer as B          # noqa: E402
import build_changer_v4 as V       # noqa: E402
import build_gripper_tool as GT    # noqa: E402

NECK_Z = (-5.0, -21.0)
REST_X = -GT.BX                    # -39: the rest face over the dock's tail rib
RIB_Z = (-25.0, -41.0)             # tool z span of the dock's tail rib


def bx(xs, ys, zs):
    return Pos(sum(xs) / 2, sum(ys) / 2, sum(zs) / 2) * Box(abs(xs[1] - xs[0]), abs(ys[1] - ys[0]), abs(zs[1] - zs[0]))


def rbox(xs, ys, zs, r_v=0.0, r_e=0.0, axis="z"):
    """Box with its edges parallel to `axis` filleted r_v, then all remaining edges r_e (0 = skip)."""
    b = bx(xs, ys, zs)
    ax = {"x": Axis.X, "y": Axis.Y, "z": Axis.Z}[axis]
    if r_v > 0:
        b = fillet(b.edges().filter_by(ax), r_v)
    if r_e > 0:
        b = fillet(b.edges(), r_e)
    return b


def coupling():
    """The coupling half's separate parts (identical to GT1)."""
    return {"plate": GT.coupling_plate(), "pins": B.pins(), "rotor": V.rotor_v4(), "shaft": GT.shaft()}


def neck_core():
    """GT1's neck exactly (z -5..-21): 36 sq dock key, rotor seat, shaft bearing, 4 inserts, flush neck magnet pocket."""
    return insert_leads(GT.body() & bx((-20, 20), (-20, 20), NECK_Z), neck_insert_mouths())


# ---------------------------------------------------------------- hardware positions (tool frame) for the viewer / gates
def coupling_hardware():
    """[(id, spec, pos, axis(out of the part = toward the head), length, kind)] for the coupling half."""
    hw = []
    for k in range(4):
        a = radians(45 + 90 * k)
        x, y = GT.GT_SCREW_R * cos(a), GT.GT_SCREW_R * sin(a)
        hw.append((f"plate_screw{k + 1}", "M3x10 countersunk", (x, y, 0.0), (0, 0, 1), 10.0, "bolt"))
        hw.append((f"neck_insert{k + 1}", "M3 insert 4.0 bore x 6.7", (x, y, -5.0), (0, 0, 1), 5.7, "insert"))
    for k in range(3):
        a = radians(120 * k + B.PIN_PHASE)
        r0, r1 = B.PIN_RIN, B.PIN_RIN + B.PIN_L
        hw.append((f"pin{k + 1}", "3x12 steel dowel", (r1 * cos(a), r1 * sin(a), B.TOOL_PIN_Z), (cos(a), sin(a), 0), 12.0, "pin"))
    hw.append(("neck_magnet", "N52 6x3 magnet", (-B.NECK_SQ / 2, 0.0, V.WASHER_Z), (-1, 0, 0), 3.0, "magnet"))
    return hw


def engrave_py(shape, text, x, z, y_face, size=5.0, depth=0.4):
    """Cut `text` 0.4 deep into a +y face at y = y_face, reading left-to-right seen from +y (z up)."""
    from build123d import Plane, Text, extrude
    try:
        pl = Plane(origin=(x, y_face - depth, z), x_dir=(-1, 0, 0), z_dir=(0, 1, 0))
        t = pl * extrude(Text(text, font_size=size), amount=2 * depth)
        return shape - t
    except Exception as e:
        print("engraving skipped:", e)
        return shape


# Insert lead-in (owner 10/28 via pm-eng): every heat-set insert hole gets a shallow locating counterbore at its mouth so the
# insert stands up straight before the iron pushes it in.  M3: kit knurl OD ~4.6 (FFVRVSS, not yet measured) + 0.15.
INSERT_LEAD = dict(d=4.75, depth=0.4)          # "just a hair" (owner 10/28): 0.4 deep until the coupon is measured


def insert_leads(shape, mouths):
    """mouths: [(x, y, z_mouth, sign)] with sign = +1 if the hole goes up (+z) from the mouth, -1 if it goes down."""
    d, h = INSERT_LEAD["d"], INSERT_LEAD["depth"]
    for x, y, z, sg in mouths:
        shape -= Pos(x, y, z + sg * (h / 2 - 0.01)) * Cylinder(d / 2, h)
    return shape


def neck_insert_mouths():
    return [(GT.GT_SCREW_R * cos(radians(45 + 90 * k)), GT.GT_SCREW_R * sin(radians(45 + 90 * k)), -5.0, -1) for k in range(4)]

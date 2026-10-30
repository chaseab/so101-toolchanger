# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""R3 fix verification: (1) blade vs rotor at ENTRY with the rotor floated 1.0 mm across the slot (worst case) -> the
blade tip must land inside the lead-in chamfer, not on the rotor top; (2) carrier (button-head pockets) vs spigot
at entry/crest/lock; (3) skirt vs ring overlap (should be 0 apart from the glued collar contact)."""
import json
from build123d import Pos, Rot, Cylinder, Box
import build_changer as B
car = B.to_gripper(B.carrier())
plate = B.coupling_plate(); rot = B.rotor()
out = {}
for name, (r, zf) in {"entry": (-45, B.Z_TOOL_FACE_ENTRY), "crest": (-6, B.Z_TOOL_FACE_ENTRY + B.LIFT_CREST), "lock": (0, B.Z_TOOL_FACE_LOCK)}.items():
    t = Pos(B.HORN_CX, 0, zf) * Rot(0, 0, r) * plate
    out[f"{name}: carrier x plate"] = round((car & t).volume, 3)
# button heads (dia 5.7, h 1.65) sitting in the 1.7 pockets
heads = None
for sx in (-5, 5):
    for sy in (-5, 5):
        h = Pos(B.HORN_CX + sx, sy, B.Z_HORN_BOT - B.CAR_FL_T + 1.7 - 1.65 / 2) * Cylinder(2.85, 1.65)
        heads = h if heads is None else heads + h
for name, zf in {"entry": B.Z_TOOL_FACE_ENTRY, "crest": B.Z_TOOL_FACE_ENTRY + B.LIFT_CREST}.items():
    out[f"{name}: button heads x plate"] = round((heads & (Pos(B.HORN_CX, 0, zf) * plate)).volume, 3)
# blade at entry: the blade is still relative to the TOOL during the twist; rotor floated 1.0 across the slot (x)
for dx in (0.0, 1.0, -1.0):
    rt = Pos(B.HORN_CX + dx, 0, B.Z_TOOL_FACE_ENTRY) * rot
    v = (car & rt).volume
    # is the blade tip over the opening? sample: the tip line must lie within the chamfered mouth at the rotor top
    mouth = B.SLOT_W / 2 + B.SLOT_LEAD
    out[f"entry, rotor float {dx:+.1f}: blade x rotor mm3"] = round(v, 3)
    out[f"entry, rotor float {dx:+.1f}: tip half-width 0.5 inside mouth half {mouth:.1f} after offset"] = abs(dx) + 0.5 <= mouth
sk, rg = B.to_gripper(B.skirt()), B.to_gripper(B.ring())
out["skirt x ring mm3"] = round((sk & rg).volume, 3)
bb = sk.bounding_box(); out["skirt top z"] = round(bb.max.Z, 2); out["ring shoulder z"] = B.Z_LIP_BOT + 3.0
print(json.dumps(out, indent=1))

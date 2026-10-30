# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Interference check (exact OCC booleans), master vs tool parts at ENTRY, mid-ramp, CREST and LOCK (gripper frame).
Also: blade vs rotor slot at lock.  Writes interference.json."""
import json
from pathlib import Path
from build123d import Pos, Rot
import build_changer as B
mp = B.master_parts()
M = {k: B.to_gripper(v) for k, v in mp.items() if k in ("cage", "ring", "skirt", "carrier", "dowels", "ball0", "ball1", "ball2", "plunger0", "plunger1", "plunger2")}
T = {"plate": B.coupling_plate(), "pins": B.pins(), "rotor": B.rotor()}
# master frame rotation of the TOOL relative to lock: pins at master angle PHI_V + rot
states = {"entry": (-45, B.Z_TOOL_FACE_ENTRY), "ramp_27": (-18, B.Z_TOOL_FACE_ENTRY + 0.45),
          "crest_39": (-6, B.Z_TOOL_FACE_ENTRY + B.LIFT_CREST), "lock": (0, B.Z_TOOL_FACE_LOCK)}
out = {}
for s, (rot, zf) in states.items():
    for tn, tm in T.items():
        if tn == "rotor" and s != "lock": continue          # rotor turns with the blade during the twist
        t = Pos(B.HORN_CX, 0, zf) * Rot(0, 0, rot) * tm
        for mn, mm in M.items():
            v = (t & mm).volume
            if v > 0.01: out[f"{s}: tool {tn} x master {mn}"] = round(v, 3)
print(json.dumps(out, indent=1) if out else "no interference > 0.01 mm3 in any state")
(Path(__file__).parent / "interference.json").write_text(json.dumps({"states": states, "overlaps_mm3": out}, indent=1))

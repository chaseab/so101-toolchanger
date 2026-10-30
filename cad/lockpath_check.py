# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Lock-path check on the exported ring mesh: for each twist angle, the lowest pin-centre height (above the lip
top) at which a 3 mm radial pin clears the ring.  Confirms entry, ramp, crest, V seat and stop on real geometry."""
import json, numpy as np, trimesh
from pathlib import Path
import build_changer as B
H = Path(__file__).resolve().parent
ring = trimesh.util.concatenate([trimesh.load(H / "stl/master_ring.stl"), trimesh.load(H / "stl/master_dowels.stl")])
ring.apply_translation([-B.HORN_CX, 0, 0])
rmax, rpin = B.PIN_RIN + B.PIN_L, B.PIN_D / 2
rs = np.linspace(B.R_BORE + 0.2, rmax, 16)
ts = np.linspace(-rpin * 0.98, rpin * 0.98, 13)
rows = []
for phi in np.arange(-20, 66.5, 0.5):
    a = np.radians(phi)
    ur, ut = np.array([np.cos(a), np.sin(a), 0]), np.array([-np.sin(a), np.cos(a), 0])
    O = np.array([r * ur + t * ut + [0, 0, B.Z_LIP_TOP + 9.0] for r in rs for t in ts])
    D = np.tile([0, 0, -1.0], (len(O), 1))
    loc, idx, _ = ring.ray.intersects_location(O, D, multiple_hits=False)
    need = -1e9
    hit = np.zeros(len(O), bool); hit[idx] = True
    for (p, i) in zip(loc, idx):
        t = ts[i % len(ts)]
        need = max(need, p[2] - B.Z_LIP_TOP + np.sqrt(rpin**2 - t**2))
    rows.append((float(phi), None if need < -1e8 else round(float(need), 3), int(hit.sum())))
out = {"pin_centre_needed_above_lip_top": rows,
       "design": {"entry": B.PIN_C_ENTRY, "crest": B.PIN_C_CREST, "lock": round(B.PIN_C_LOCK, 3)}}
(H / "lockpath.json").write_text(json.dumps(out))
for phi, need, n in rows:
    if phi % 2.5 == 0: print(f"{phi:6.1f} deg  need {need}  (rays hit {n})")

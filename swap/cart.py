# SPDX-License-Identifier: GPL-3.0-or-later
"""Cartesian nudge of the master face, tool axis held.  python cart.py DX DY DZ (mm, base frame) | axis D (mm along tool) [--dry]"""
import subprocess, sys
import numpy as np
import real_io as R
from where import d5_from_ticks, CAL, KEYS
from plan import ik, ticks_from_d5
sys.path.insert(0, ".")
from scservo_sdk import PortHandler, PacketHandler
p = PortHandler("COM8"); p.openPort(); p.setBaudRate(1000000); h = PacketHandler(0)
t = [h.read2ByteTxRx(p, i, 56)[0] for i in range(1, 6)]; p.closePort()
d0 = d5_from_ticks(t); p0, R0 = R.face_pose(d0); ax = -R0[:, 2]
a = sys.argv[1:]; dry = "--dry" in a; a = [x for x in a if x != "--dry"]
d = ax * float(a[1]) / 1000 if a[0] == "axis" else np.array([float(v) for v in a[:3]]) / 1000
d1, err, aerr = ik(p0 + d, ax, d0)
d1[4] = d0[4]
t1 = ticks_from_d5(d1)
p1, _ = R.face_pose(d1)
print("face", np.round(p0 * 1000, 1), "->", np.round(p1 * 1000, 1), f"(ik err {err:.1f} mm, axis {aerr:.1f} deg)", "ticks", t, "->", t1)
if err > 3 or aerr > 3: sys.exit("IK miss, not moving")
if not dry:
    import capguard; capguard.require("cart.py")
    args = []
    for n, v0, v1 in zip(["pan", "lift", "elbow", "flex"], t, t1):
        if v0 != v1: args += [n, str(v1)]
    subprocess.run([sys.executable, "jog.py", "goto"] + args)

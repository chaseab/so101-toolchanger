# SPDX-License-Identifier: GPL-3.0-or-later
"""IK for the master face: position + tool axis (roll held).  python plan.py X Y Z AX AY AZ [seed ticks x5]"""
import sys, json, os
import numpy as np
import real_io as R
from where import CAL, KEYS, d5_from_ticks

def ticks_from_d5(d5):
    return [int(round(d * 4095 / 360 + (CAL[k]["range_min"] + CAL[k]["range_max"]) / 2)) for d, k in zip(d5, KEYS)]

def ik(p_t, axis_t, d0, iters=3000):
    x = np.radians(np.asarray(d0, float)); h = 1e-6
    tool = lambda Rm: -Rm[:, 2]
    for _ in range(iters):
        p, Rm = R.face_pose(np.degrees(x))
        e = np.concatenate([p_t - p, 0.3 * np.cross(tool(Rm), axis_t)])
        if np.linalg.norm(e) < 1e-7: break
        J = np.zeros((6, 4))
        for i in range(4):
            xi = x.copy(); xi[i] += h
            p2, R2 = R.face_pose(np.degrees(xi))
            e2 = np.concatenate([p_t - p2, 0.3 * np.cross(tool(R2), axis_t)])
            J[:, i] = -(e2 - e) / h
        dx = J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e)
        x[:4] += np.clip(dx, -0.05, 0.05)
    p, Rm = R.face_pose(np.degrees(x))
    return np.degrees(x), np.linalg.norm(p_t - p) * 1000, np.degrees(np.arccos(np.clip(tool(Rm) @ axis_t, -1, 1)))

if __name__ == "__main__":
    a = [float(v) for v in sys.argv[1:]]
    p_t = np.array(a[:3]) / 1000; ax = np.array(a[3:6]); ax /= np.linalg.norm(ax)
    seed = a[6:11] if len(a) >= 11 else [2005, 895, 3075, 2321, 2000]
    d, err, aerr = ik(p_t, ax, d5_from_ticks(seed))
    t = ticks_from_d5(d)
    ok = all(CAL[k]["range_min"] <= v <= CAL[k]["range_max"] for v, k in zip(t, KEYS))
    print("deg", np.round(d, 1), "ticks", t, f"pos err {err:.1f} mm, axis err {aerr:.1f} deg", "IN RANGE" if ok else "OUT OF RANGE")

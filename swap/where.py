# SPDX-License-Identifier: GPL-3.0-or-later
"""Master-face position from raw follower ticks (FK via real_io.face_pose).  python where.py [pan lift elbow flex roll]"""
import json, os, sys
import numpy as np
sys.argv += []
import real_io as R
CAL = json.load(open(os.path.expanduser(r"~/.cache/huggingface/lerobot/calibration/robots/so_follower/follower_arm.json")))
KEYS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

def d5_from_ticks(t):
    return [(t[k] - (CAL[k]["range_min"] + CAL[k]["range_max"]) / 2) * 360 / 4095 for k in range(5) for k in [KEYS[k]]][:5] if False else \
           [(t[i] - (CAL[k]["range_min"] + CAL[k]["range_max"]) / 2) * 360 / 4095 for i, k in enumerate(KEYS)]

def where(t):
    d5 = d5_from_ticks(t)
    p, Rm = R.face_pose(d5)
    return d5, p * 1000, Rm[:, 2]

if __name__ == "__main__":
    t = [float(x) for x in sys.argv[1:6]]
    d5, p, z = where(t)
    print("deg", np.round(d5, 1), "face mm", np.round(p, 1), "tool axis(+z of gripper)", np.round(z, 2))

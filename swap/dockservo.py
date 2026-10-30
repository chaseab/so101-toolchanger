# SPDX-License-Identifier: GPL-3.0-or-later
"""Dock-cam visual servo of the master onto a taught seated frame (10/29).

The dock cam is bolted to the dock, so the master's image position in it is a direct measure of the master pose in the
dock frame. Reference = a dock-cam frame (800 px wide) taken while the master was really in the bore (s5, 18:40).
Encoders cannot do this: with the master in the bore the links flex ~9 mm, so the same joint angles are not the same
physical pose (10/29).

    python dockservo.py match REF.jpg [--box x0,y0,x1,y1]     template of the master in REF -> shift + scale in a live frame
"""
import os, subprocess, sys
import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
BOX = (300, 70, 560, 270)        # master (skirt + body) in the s5 reference, 800 px frame


def grab_dock(path):
    import capture as C
    alt = C.find_cameras()["scene"]
    subprocess.run([C.FF, "-y", "-loglevel", "error", *C.cam_args(alt), "-frames:v", "10", "-update", "1",
                    "-vf", "scale=800:-1", path])
    return cv2.imread(path)


def match(ref, live, box=BOX):
    """Best (dx, dy, scale, score) of the ref master template in the live frame, scales 0.85..1.15."""
    x0, y0, x1, y1 = box
    t0 = cv2.cvtColor(ref[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    g = cv2.cvtColor(live, cv2.COLOR_BGR2GRAY)
    best = None
    for s in np.linspace(0.85, 1.15, 13):
        t = cv2.resize(t0, None, fx=s, fy=s)
        r = cv2.matchTemplate(g, t, cv2.TM_CCOEFF_NORMED)
        _, mx, _, loc = cv2.minMaxLoc(r)
        cx = loc[0] + t.shape[1] / 2 - (x0 + x1) / 2
        cy = loc[1] + t.shape[0] / 2 - (y0 + y1) / 2
        if best is None or mx > best[3]:
            best = (cx, cy, s, mx)
    return best


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[0] == "match":
        ref = cv2.imread(a[1]); live = grab_dock(os.path.join(HERE, "logs", "vis", "dock_live.jpg"))
        dx, dy, s, sc = match(ref, live)
        print(f"master vs ref: dx {dx:+.1f} px  dy {dy:+.1f} px  scale {s:.3f}  score {sc:.3f}")

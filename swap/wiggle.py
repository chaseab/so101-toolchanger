# SPDX-License-Identifier: GPL-3.0-or-later
"""Does the parked tool move WITH the master? (owner 10/29: "get in the habit of moving it around after you've gotten
it on".) Load numbers fooled us (a 16 % roll 'ramp' that was not a lock); the dock cam does not lie.

    python wiggle.py [--dz 1.5] [--dy 0] [--thr 3]
Grabs a dock-cam frame, moves the master by (0, dy, dz) mm with smooth.py, grabs again, moves back, and template-matches
the tool body region (left of the dock wall) between the two frames. Prints the tool shift in px and FOLLOWS / STILL.
Exit 0 = follows, 2 = still.
"""
import os, subprocess, sys
import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import capture as C

a = sys.argv[1:]
def opt(n, d): return float(a[a.index(n) + 1]) if n in a else d
DZ, DY, THR = opt("--dz", 4.0), opt("--dy", 0.0), opt("--thr", 8.0)   # coupled tool shifts ~15 px for 4 mm; 3.6 px was noise (10/29)
TOOL_BOX = (40, 250, 230, 390)          # tool body / fingers in the 800 px dock frame (left of the wall)
PY = os.environ.get("TC_PYTHON", "python")


import capguard
capguard.require("wiggle.py")


def grab(name):
    p = os.path.join(HERE, "logs", "vis", name)
    if capguard.live_take():                       # camera busy recording: read the newest frame of the take
        capguard.latest_frame(p, 800); return cv2.imread(p, 0)
    alt = C.find_cameras()["scene"]
    subprocess.run([C.FF, "-y", "-loglevel", "error", *C.cam_args(alt), "-frames:v", "8", "-update", "1", "-vf", "scale=800:-1", p])
    return cv2.imread(p, 0)


def shift(a_img, b_img):
    x0, y0, x1, y1 = TOOL_BOX
    t = a_img[y0:y1, x0:x1]
    r = cv2.matchTemplate(b_img, t, cv2.TM_CCOEFF_NORMED)
    _, mx, _, loc = cv2.minMaxLoc(r)
    return loc[0] - x0, loc[1] - y0, mx


def sm(*args):
    r = subprocess.run([PY, os.path.join(HERE, "smooth.py"), *map(str, args)], capture_output=True, text=True, cwd=HERE)
    return (r.stdout.strip().splitlines() or ["?"])[-1][:90]


A = grab("wig_a.jpg")
print("move:", sm("xyz", 0, DY, DZ, "--speed", 1.5))
B = grab("wig_b.jpg")
print("back:", sm("xyz", 0, -DY, -DZ, "--speed", 1.5))
dx, dy, sc = shift(A, B)
mag = float(np.hypot(dx, dy))
fol = mag >= THR
print(f"tool shift {dx:+d},{dy:+d} px (|{mag:.1f}|, match {sc:.2f}) -> {'FOLLOWS' if fol else 'STILL'}")
sys.exit(0 if fol else 2)

# SPDX-License-Identifier: GPL-3.0-or-later
"""Put a carried tool back into its dock slot when the first drop lands on the wall top (10/29).

Sideways (x) is invisible to the dock cam and pan is the only sideways joint, so search pan offsets. For each offset:
raise clear, pan, lower gently (low lift lead = light push), then check the dock cam: the tool region moves DOWN by
>= 12 px when the neck drops into the slot; on the wall top it does not move (the arm only flexes).

    python redock.py [--pans 0,5,-5,10,-10,15,-15] [--down 22] [--up 14]
"""
import os, subprocess, sys, time
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import capture as C

PY = os.environ.get("TC_PYTHON", "python")
a = sys.argv[1:]
def opt(n, d): return a[a.index(n) + 1] if n in a else d
PANS = [int(v) for v in opt("--pans", "0,5,-5,10,-10,15,-15").split(",")]
DOWN, UP = float(opt("--down", 22)), float(opt("--up", 14))
BOX = (20, 60, 200, 200)          # tool body above/left of the wall in the 640 px dock frame


import capguard
capguard.require("redock.py")


def grab(n):
    p = os.path.join(HERE, "logs", "vis", n)
    if capguard.live_take():
        capguard.latest_frame(p, 640); return cv2.imread(p, 0)
    subprocess.run([C.FF, "-y", "-loglevel", "error", *C.cam_args(C.find_cameras()["scene"]), "-frames:v", "8", "-update", "1",
                    "-vf", "scale=640:-1", p])
    return cv2.imread(p, 0)


def run(*args):
    r = subprocess.run([PY, *map(str, args)], capture_output=True, text=True, cwd=HERE)
    return (r.stdout.strip().splitlines() or [r.stderr.strip()[-150:]])[-1][:110]


def pan_by(dt):
    code = (f"import jog as J, time\np=J.rd(1,56,2); g=p+({dt}); J.wr(1,46,2,40)\n"
            f"J.wr(1,42,2,g+(6 if {dt}>0 else -6)); time.sleep(1.0); J.wr(1,42,2,g); time.sleep(0.8); print('pan', p, '->', J.rd(1,56,2))")
    r = subprocess.run([PY, "-c", code], capture_output=True, text=True, cwd=HERE)
    return r.stdout.strip()


ref = grab("rd_ref.jpg")
x0, y0, x1, y1 = BOX
t = ref[y0:y1, x0:x1]
last = 0
for dp in PANS:
    print(f"--- pan offset {dp:+d} ticks")
    print("  up:", run("liftout.py", UP, "--pitch", 25.0, "--step", 1.0))
    print(" ", pan_by(dp - last)); last = dp
    print("  down:", run("liftout.py", -DOWN, "--pitch", 25.0, "--step", 1.0, "--lead", 5))
    im = grab(f"rd_{dp:+d}.jpg")
    r = cv2.matchTemplate(im, t, cv2.TM_CCOEFF_NORMED); _, mx, _, loc = cv2.minMaxLoc(r)
    dy = loc[1] - y0
    print(f"  tool moved {loc[0] - x0:+d},{dy:+d} px (match {mx:.2f})")
    if dy >= 12:
        print(f"IN SLOT at pan offset {dp:+d}"); sys.exit(0)
print("no slot entry in the pan search"); sys.exit(2)

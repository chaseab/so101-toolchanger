# SPDX-License-Identifier: GPL-3.0-or-later
"""Closed-loop vertical lift of the master face, tool pitch held from MEASURED angles (10/29).

cart.py sends lift + flex goals together; under load the lift lags ~3 deg while flex arrives, so the face pitches and
levers the tool (10/29 18:3x: face went 6 mm DOWN, 5.7 deg nose-up).  Here: lift steps first, then flex is set from the
measured lift + elbow so (lift + elbow + flex) stays at its start value (= tool pitch, checked by FK), repeat.

    python liftout.py DZ_MM [--step 1.5] [--dry]      (raises the face by DZ in base z; stops on stall / 90 % load / temp)
"""
import sys, time
import numpy as np
import real_io as R
from where import d5_from_ticks, CAL, KEYS
import jog as J

TPD = 4096 / 360.0
LEAD = float(sys.argv[sys.argv.index("--lead") + 1]) if "--lead" in sys.argv else 12.0   # max deg the lift goal may run ahead: the servo gives only ~5 % load per deg of error (10/29)
a = sys.argv[1:]
dry = "--dry" in a
step = float(a[a.index("--step") + 1]) if "--step" in a else 1.5
opt = {a[i + 1] for i, x in enumerate(a) if x in ("--step", "--pitch", "--lead")}
dz = float([x for x in a if not x.startswith("--") and x not in opt][0])


def ticks():
    return [J.rd(i, 56, 2) for i in range(1, 6)]


def fk(t):
    p, Rm = R.face_pose(d5_from_ticks(t))
    return p * 1000, Rm[:, 2]


t0 = ticks()
p0, z0 = fk(t0)
d0 = d5_from_ticks(t0)
pitch0 = d0[1] + d0[2] + d0[3]
if "--pitch" in a:                                       # level to a given pitch-sum (10/29 seat: 25.3)
    pitch0 = float(a[a.index("--pitch") + 1])
print(f"start face {np.round(p0, 1)} axis {np.round(z0, 2)} pitch-sum {pitch0:.1f}")
# which lift direction raises the face: probe FK with a 1 deg change, flex compensated
probe = list(t0); probe[1] -= TPD; probe[3] += TPD
up = -1 if fk(probe)[0][2] > p0[2] else +1
sgn = 1 if dz >= 0 else -1
up *= sgn                                                # negative DZ = lower (put a tool back in its slot)
print("lift direction for up:", "decreasing ticks" if up < 0 else "increasing ticks")
target = p0[2] + dz
if not dry:
    import capguard; capguard.require("liftout.py")
    J.start_trace("liftout")
for it in range(60):
    t = ticks(); p, z = fk(t)
    if (p[2] - target) * sgn >= -0.5:
        print(f"reached: face {np.round(p, 1)} axis {np.round(z, 2)}"); break
    d = d5_from_ticks(t)
    # goal accumulates (so a resisted lift pushes harder), but never more than 6 deg ahead of where the joint is
    prev = locals().get("lift_goal", t[1])
    lift_goal = prev + up * int(step * TPD)
    if abs(lift_goal - t[1]) > LEAD * TPD:
        lift_goal = t[1] + up * int(LEAD * TPD)
    if dry:
        print(f"[dry] it {it}: face z {p[2]:.1f} -> lift goal {lift_goal}"); break
    J.wr(2, 41, 1, 10); J.wr(2, 46, 2, 60); J.wr(2, 42, 2, lift_goal)
    time.sleep(0.6)
    t = ticks(); d = d5_from_ticks(t)
    flex_deg = pitch0 - d[1] - d[2]                      # keep lift + elbow + flex constant
    flex_goal = int(round(flex_deg * TPD + (CAL[KEYS[3]]["range_min"] + CAL[KEYS[3]]["range_max"]) / 2))
    J.wr(4, 46, 2, 60); J.wr(4, 42, 2, flex_goal)
    time.sleep(0.5)
    t = ticks(); p, z = fk(t)
    ld = {n: J.load(k + 1) for k, n in enumerate(["pan", "lift", "elbow", "flex", "roll"])}
    tmp = max(J.rd(k, 63, 1) for k in range(1, 6))
    if tmp >= 55:                                        # 2 reads in a row (10/27 + 10/29: single 67-78 C glitch reads)
        time.sleep(0.2); tmp = min(tmp, max(J.rd(k, 63, 1) for k in range(1, 6)))
    print(f"it {it}: lift {t[1]} (goal {lift_goal})  face z {p[2]:6.1f} (+{p[2] - p0[2]:4.1f})  y {p[1]:6.1f}  "
          f"axis z {z[2]:+.2f}  load lift {ld['lift']:+.0f}% flex {ld['flex']:+.0f}% elbow {ld['elbow']:+.0f}%  {tmp}C")
    moved = abs(t[1] - locals().get("last_lift", t[1] + 99))
    last_lift = t[1]
    stuck = locals().get("stuck", 0) + 1 if moved < 3 and abs(lift_goal - t[1]) >= (LEAD - 1) * TPD else 0
    if stuck >= 3:                                       # resisted: stop before the 80 %/2 s overload protection trips
        J.hold(); print("lift not moving for 3 steps -> HOLD (" + ("resisted" if sgn > 0 else "bottomed") + ")"); break
    if abs(ld["lift"]) >= 70 or abs(ld["elbow"]) >= 70 or tmp >= 55:
        J.hold(); print("LOAD/TEMP -> HOLD"); break
    if it == 0:
        zref = z[2]                                      # axis after the first flex set = the commanded pitch
    if abs(z[2] - zref) > 0.06:                          # drifted > ~3.4 deg from the commanded pitch (was: off level)
        J.hold(); print("pitch drifted > 3.4 deg -> HOLD"); break
else:
    J.hold(); print("60 iterations -> HOLD")

# SPDX-License-Identifier: GPL-3.0-or-later
"""Smooth straight-line move of the master face (10/29). Replaces cart.py / liftout.py stepping for dock work.

    python smooth.py z DZ            [--speed 4] [--pan-soft 30] [--pitch-from-seat] [--dry]
    python smooth.py axis D          (+ = along the tool's -z, i.e. away from the base; - = back toward the base)
    python smooth.py xyz DX DY DZ    (base frame, mm)
    python smooth.py to X Y Z        (absolute face target, base frame mm; pitch blended to the 10/29 seat pitch)

How it is smooth:
- the whole path is IK-planned BEFORE moving (every 0.5 mm), tool axis held, roll held; minimum-jerk timing at --speed mm/s
- streamed at ~30 Hz as servo goals with an outer integral loop per joint (goal = plan + I, I += ki * (plan - measured)),
  so gravity sag / slot friction are taken up continuously instead of by 8-10 deg lead jumps (10/29 liftout stick-slip).
  I starts from the servo's present (Goal - Present) error, so the first tick does not jump.
- lift + elbow + flex move together every tick (pitch never leads/lags by a whole step like liftout.py)
- --pan-soft N: pan Torque_Limit N % during the move so the dock slot can steer the tool sideways along the arc
  (a slightly twisted dock; the SO-101 has no wrist yaw). Restored to 80 % at the end.
--contact E: stop (hold) as soon as lift/elbow/flex lag the plan by > E deg = the face met something (insert to depth).
Stops and HOLDS on: any |load| >= 75 % for 0.5 s, temp >= 50 C, |plan - measured| > 12 deg (lift) / 8 deg (others),
tool pitch off the plan by > 3 deg (the slot jam limit is ~3.8), Ctrl-C, or the swap/STOP file.
Logs every tick to logs/smooth_<time>.csv (plan, measured, goal, load) for the encoder review.
"""
import csv, os, sys, time
import numpy as np
import real_io as R
from where import d5_from_ticks, CAL, KEYS
from plan import ik, ticks_from_d5
import jog as J

HERE = os.path.dirname(os.path.abspath(__file__))
TPD = 4096 / 360.0
NJ = 4                                   # pan lift elbow flex are planned; roll is held
KI = np.array([2.0, 4.0, 3.0, 3.0])      # 1/s outer-loop integral gains
ILIM = np.array([3.0, 12.0, 8.0, 6.0])   # deg: max integral (= the lead the joint may need; lift needs ~10 under the changer)
ERRLIM = np.array([6.0, 12.0, 8.0, 8.0])
SEAT_D5 = [-87.0, 10.5, 63.9, -49.1]     # 10/29 18:29 teleop seat (pitch reference, --pitch-from-seat)


def read_t():
    return np.array([J.rd(i, 56, 2) for i in range(1, 6)], float)


def pitch_err(d_plan, d_meas):
    _, Rp = R.face_pose(d_plan); _, Rm = R.face_pose(d_meas)
    return np.degrees(np.arccos(np.clip(Rp[:, 2] @ Rm[:, 2], -1, 1)))


def main():
    a = sys.argv[1:]
    dry = "--dry" in a
    def opt(name, default):
        return float(a[a.index(name) + 1]) if name in a else default
    speed, pan_soft, contact = opt("--speed", 4.0), opt("--pan-soft", 0), opt("--contact", 0)
    pos = [x for i, x in enumerate(a) if not x.startswith("--") and (i == 0 or a[i - 1] not in ("--speed", "--pan-soft", "--contact", "--az"))]
    mode, vals = pos[0], [float(v) for v in pos[1:]]

    t0 = read_t(); d0 = d5_from_ticks(t0)
    p0, R0 = R.face_pose(d0); ax0 = -R0[:, 2]; ax = ax0
    if "--pitch-from-seat" in a or mode == "to":
        _, Rs = R.face_pose(SEAT_D5 + [d0[4]]); ax = -Rs[:, 2]
    if "--az" in a:                                      # explicit tool-axis z (pitch) for the target, e.g. -0.01
        ax = np.array([ax[0], ax[1], opt("--az", 0.0)]); ax /= np.linalg.norm(ax)
    if mode == "to":                                     # absolute face target (mm, base frame), seat pitch
        delta = np.array(vals[:3]) / 1000 - p0
    elif mode == "z":
        delta = np.array([0, 0, vals[0]]) / 1000
    elif mode == "axis":
        delta = ax * vals[0] / 1000
    else:
        delta = np.array(vals[:3]) / 1000
    L = np.linalg.norm(delta) * 1000
    n = max(2, int(L / 1.0) + 1)          # 1 mm plan spacing (IK ~1 s per point)
    plan, seed = [], list(d0)
    for s in np.linspace(0, 1, n):
        ax_s = (1 - s) * ax0 + s * ax; ax_s /= np.linalg.norm(ax_s)      # blend the pitch: no jump at tick 1
        d, err, aerr = ik(p0 + s * delta, ax_s, seed, iters=400)
        if err > 2.0 or aerr > 1.5:                       # lateral targets need a little axis yaw (no wrist yaw)
            sys.exit(f"IK miss at s={s:.2f} ({err:.1f} mm, {aerr:.1f} deg): not moving")
        d[4] = d0[4]; plan.append(d[:5]); seed = list(d)
    plan = np.array(plan)
    tk = np.array([ticks_from_d5(d) for d in plan])
    for k in range(NJ):
        c = CAL[KEYS[k]]
        if tk[:, k].min() < c["range_min"] or tk[:, k].max() > c["range_max"]:
            sys.exit(f"{KEYS[k]} leaves its calibrated range on this path: not moving")
    T = max(1.5, 1.875 * L / speed)       # min-jerk peak speed = 1.875 * mean
    print(f"path {L:.1f} mm in {T:.1f} s, face {np.round(p0*1000,1)} -> {np.round((p0+delta)*1000,1)}, "
          f"joint span (deg) {np.round(plan[-1,:4]-plan[0,:4],1)}")
    if dry:
        return
    import capguard; capguard.require("smooth.py")
    J.start_trace("smooth")

    # integral starts at the servo's standing error so tick 1 continues the hold instead of jumping
    goal0 = np.array([J.rd(i, 42, 2) for i in range(1, 5)], float)
    I = np.clip((goal0 - t0[:4]) / TPD, -ILIM, ILIM)
    for i in range(1, 5):
        J.wr(i, 41, 1, 0); J.wr(i, 46, 2, 0)        # max acceleration / speed: the stream itself is the profile
    if pan_soft:
        J.wr(1, 48, 2, int(pan_soft * 10))
    log = open(os.path.join(HERE, "logs", f"smooth_{time.strftime('%Y%m%d_%H%M%S')}.csv"), "w", newline="")
    w = csv.writer(log)
    w.writerow(["t", "s"] + [f"plan_{k}" for k in KEYS[:4]] + [f"meas_{k}" for k in KEYS[:4]] + [f"goal_{k}" for k in KEYS[:4]]
               + [f"load_{k}" for k in KEYS[:4]] + ["pitch_err", "face_x", "face_y", "face_z"])
    over = np.zeros(4); tstart = time.time(); last = tstart; why = None; peak = np.zeros(4)
    try:
        while True:
            now = time.time(); dt = now - last; last = now
            tau = min(1.0, (now - tstart) / T)
            s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5          # minimum jerk
            f = s * (n - 1); i0 = min(int(f), n - 2); fr = f - i0
            dp = plan[i0] * (1 - fr) + plan[i0 + 1] * fr
            tm = read_t(); dm = np.array(d5_from_ticks(tm))
            e = dp[:4] - dm[:4]
            I = np.clip(I + KI * e * dt, -ILIM, ILIM)
            goal = dp[:4] + I
            gt = ticks_from_d5(list(goal) + [dp[4]])
            for k in range(NJ):
                J.wr(k + 1, 42, 2, gt[k])
            ld = np.array([J.load(k + 1) for k in range(NJ)])
            peak = np.maximum(peak, np.abs(ld))
            pe = pitch_err(list(dp), list(dm))
            pf, _ = R.face_pose(list(dm))
            w.writerow([f"{now - tstart:.3f}", f"{s:.4f}", *np.round(dp[:4], 2), *np.round(dm[:4], 2), *np.round(goal, 2),
                        *ld, f"{pe:.2f}", *np.round(pf * 1000, 1)])
            over = np.where(np.abs(ld) >= 75, over + dt, 0)
            if (over >= 0.5).any():
                why = f"load >= 75 % on {[KEYS[k] for k in np.where(over >= 0.5)[0]]}"; break
            if contact and tau > 0.05 and (np.abs(e[1:]) > contact).any():
                why = f"CONTACT at s={s:.2f} (err {np.round(e, 2)} deg > {contact})"; break
            if (np.abs(e) > ERRLIM).any():
                why = f"tracking error {np.round(e, 1)} deg"; break
            if pe > 3.0:
                why = f"pitch {pe:.1f} deg off plan (jam limit ~3.8)"; break
            if os.path.exists(os.path.join(HERE, "STOP")):
                why = "STOP file"; break
            if int((now - tstart) * 2) != int((now - tstart - dt) * 2):
                if max(J.rd(k, 63, 1) for k in range(1, 6)) >= 59 and max(J.rd(k, 63, 1) for k in range(1, 6)) >= 59:   # 2 reads: single glitch reads (10/29)
                    why = "temp >= 55 C"; break
            if tau >= 1.0 and ((np.abs(e) < 0.6).all() or now - tstart > T + 2.5):
                break                                    # pan stiction can leave ~0.5 deg; the residual is printed
            time.sleep(max(0, 0.02 - (time.time() - now)))
    except KeyboardInterrupt:
        why = "Ctrl-C"
    finally:
        if why:
            J.hold()
        if pan_soft:
            J.wr(1, 48, 2, 800)
        log.close()
    tm = read_t(); pf, Rf = R.face_pose(d5_from_ticks(tm))
    print("residual (plan - meas) deg", np.round(plan[-1, :4] - np.array(d5_from_ticks(tm))[:4], 2))
    print(("HOLD: " + why) if why else "done", f"| face {np.round(pf*1000,1)} axis {np.round(-Rf[:,2],2)} | "
          f"peak load % {dict(zip(['pan','lift','elbow','flex'], peak.round(0)))} | {time.time()-tstart:.1f} s | log {log.name}")


if __name__ == "__main__":
    main()

# SPDX-License-Identifier: GPL-3.0-or-later
"""Live 'feel' view while teleoperating: where the follower disagrees with the leader, and where it is pushing on something.

    python feel.py [trace.csv]        (default: newest logs/tc_teleop_*_trace.csv, which tc_teleop.py writes at 10 Hz)

Reads only the trace file (tc_teleop owns COM8), so it never touches the bus.
Top: following error per joint = commanded - measured (deg). Free motion keeps it at ~0-2 deg; a joint that is blocked by
contact (the coupler touching the dock) or by its stop holds a steady error. Bars turn orange > 3 deg, red > 6 deg.
Middle: load % per joint (signed; new traces only). Contact shows as load rising while the error grows.
Bottom: master-face position from FK (mm, base frame) and its distance to the reference seat pose (REF below = the 10/29
18:14 teleop catch), plus the angle between the tool axes. Use it to see how close you are to the last good seat.
"""
import csv, glob, os, sys, time
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import real_io as R

HERE = os.path.dirname(os.path.abspath(__file__))
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
SHORT = ["pan", "lift", "elbow", "flex", "roll", "grip"]
REF_D5 = [-87.1, -22.7, 61.0, -33.1, 40.6]          # 10/29 18:14:2x catch (trace t+858 s), lerobot deg


def newest():
    fs = glob.glob(os.path.join(HERE, "logs", "tc_teleop_*_trace.csv"))
    return max(fs, key=os.path.getmtime)


def tail(path, n=60):
    with open(path, "rb") as f:
        f.seek(0, 2); size = f.tell(); f.seek(max(0, size - 200 * n))
        lines = f.read().decode(errors="ignore").splitlines()[1:]
    with open(path) as f:
        head = f.readline().strip().split(",")
    rows = []
    for ln in lines[-n:]:
        try:
            v = [float(x) for x in ln.split(",")]
            if len(v) == len(head):
                rows.append(v)
        except ValueError:
            pass
    return head, np.array(rows)


def main():
    fixed = sys.argv[1] if len(sys.argv) > 1 else None
    p_ref, R_ref = R.face_pose(REF_D5)
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(7, 8), gridspec_kw={"height_ratios": [3, 3, 2]})
    fig.canvas.manager.set_window_title("FEEL - follower error / load")
    while plt.fignum_exists(fig.number):
        path = fixed or newest()           # follows a restarted tc_teleop to its new trace
        try:
            head, A = tail(path)
        except OSError:
            plt.pause(0.5); continue
        if len(A) == 0:
            plt.pause(0.5); continue
        col = {h: i for i, h in enumerate(head)}
        last = A[-1]
        err = np.array([last[col["cmd_" + j]] - last[col["obs_" + j]] for j in JOINTS])
        has_load = "load_" + JOINTS[0] in col or "loadpct_" + JOINTS[0] in col
        if "loadpct_" + JOINTS[0] in col:
            load = np.array([last[col["loadpct_" + j]] for j in JOINTS])
        elif has_load:      # 18:25 trace: sign decoded twice; a true -3.2 % was written as -99.2 -> undo it
            load = np.array([-(102.4 + v) if v < 0 else v for v in (last[col["load_" + j]] for j in JOINTS)])
        else:
            load = np.zeros(6)
        age = time.time() - last[0]
        for ax in (a1, a2, a3):
            ax.clear()
        c = ["tab:red" if abs(e) > 6 else "tab:orange" if abs(e) > 3 else "tab:green" for e in err]
        a1.barh(SHORT, err, color=c); a1.set_xlim(-20, 20); a1.axvline(0, color="k", lw=0.5)
        a1.set_title(f"following error cmd - meas (deg)   {'LIVE' if age < 2 else f'STALE {age:.0f}s'}")
        for k, e in enumerate(err):
            a1.text(19, k, f"{e:+.1f}", ha="right", va="center")
        cl = ["tab:red" if abs(v) > 70 else "tab:orange" if abs(v) > 40 else "tab:blue" for v in load]
        a2.barh(SHORT, load, color=cl); a2.set_xlim(-100, 100); a2.axvline(0, color="k", lw=0.5)
        a2.set_title("load % (signed)" if has_load else "load: not in this trace (restart tc_teleop for 10 Hz load)")
        for k, v in enumerate(load):
            a2.text(98, k, f"{v:+.0f}", ha="right", va="center")
        d5 = [last[col["obs_" + j]] for j in JOINTS[:5]]
        p, Rm = R.face_pose(d5)
        dp = (p - p_ref) * 1000
        ang = np.degrees(np.arccos(np.clip(Rm[:, 2] @ R_ref[:, 2], -1, 1)))
        droll = d5[4] - REF_D5[4]
        a3.axis("off")
        a3.text(0, 0.8, f"face mm  x {p[0]*1000:6.0f}  y {p[1]*1000:6.0f}  z {p[2]*1000:6.0f}", family="monospace", fontsize=11)
        a3.text(0, 0.5, f"to ref seat  dx {dp[0]:+5.0f}  dy {dp[1]:+5.0f}  dz {dp[2]:+5.0f}  |d| {np.linalg.norm(dp):4.0f} mm",
                family="monospace", fontsize=11)
        a3.text(0, 0.2, f"tool axis off ref {ang:4.1f} deg   roll vs ref {droll:+5.1f} deg", family="monospace", fontsize=11)
        fig.tight_layout()
        plt.pause(0.15)


if __name__ == "__main__":
    main()

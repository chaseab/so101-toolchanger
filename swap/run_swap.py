# SPDX-License-Identifier: GPL-3.0-or-later
"""SO-101 tool changer: the one entry point for everything that touches the real arm (and its dry runs).

Run in the lerobot venv from toolchanger/swap:   python run_swap.py ...

    check                 offline audit: calibration, dock file, kinematics bridge, every waypoint, approach/park legs
    check --real          + READ-ONLY bus checks (protection registers, Torque_Limit, temperatures), arm at rest
    twist                 roll-only lock/unlock twists at a Torque_Limit ladder, arm held at the service pose (C2)
    friction              calibrate_friction, 2 runs, no contact (D4); saved into the dock file
    teach --dock NAME     arm LIMP, seat the tool by hand, 3 recordings (D3); saved with the calibration fingerprint
    swap --drop NAME      one drop     | --pick NAME  one pick   | --cycle N --dock NAME  N x (drop + pick)
    stop                  E-STOP from a second terminal (creates swap/STOP: the arm holds where it is)
    stopbutton            big always-on-top STOP button window (same STOP file)

Every motion command: --dry-run runs it on a fake bus (twist / friction / teach: kinematic arm; swap: the MuJoCo
twin) - no serial port.  Without --dry-run it asks you to type MOVE first.  Defaults are the safe ones: --speed 0.5,
gripper capped for the whole session, roll 85 % only inside the twist.  E-stop: Ctrl-C in this window or `stop` /
the stop button: the arm HOLDS torque in place (never parks into a dock); then you choose PARK / LIMP / exit holding.
The 12 V switch drops the arm: use it only with a hand under the arm.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import os  # noqa: E402
# dry-run twin only: dock face 310 mm out (10/24 finding: at 274 mm the routine's away pose F is past the follower's
# joint range with the 10/24 master).  The real dock pose always comes from teach.
os.environ.setdefault("TC_SEAT_R_MM", "310")
sys.path.insert(0, str(HERE.parent / "sim"))

import numpy as np  # noqa: E402

import jobs  # noqa: E402
import real_io as R  # noqa: E402
import swap_routine as SR  # noqa: E402
import tool_session  # noqa: E402
from real_io import J, sweep  # noqa: E402

DOCKS = HERE / "docks.json"
REST = np.array([sweep.REST_EXPECTED[j] for j in J])
HELD_WARNING = ("\n*** TORQUE IS HELD. Do NOT start any lerobot / run_swap / sweep command until the arm (and any tool "
                "in the dock) is SUPPORTED by hand: every connect releases torque while it configures. ***")


def calib_path(rid: str) -> Path:
    return tool_session.CALIB.parent / f"{rid}.json"


def fingerprint(rid: str):
    return tool_session.calibration_fingerprint(calib_path(rid))


def load_docks(rid: str, path: Path = DOCKS, need: bool = True):
    if not path.exists():
        if need:
            raise SR.SwapError(f"no dock file {path}: run `run_swap.py teach --dock NAME` first")
        return {}, SR.FrictionCal()
    docks, fric = SR.load(path, fingerprint(rid))
    for d in docks.values():                      # weight baselines are per session (drift): never reuse old ones
        d.load_no_tool, d.load_tool = None, None
    return docks, fric


def ask_tty(prompt: str):
    try:
        return input(prompt)
    except (EOFError, KeyboardInterrupt):
        return None                      # callers treat None as the safe answer (hold / quit)


def confirm_move(what: str) -> bool:
    print(f"\nREAL ARM: {what}\nChecklist: 12 V on, arm at its start pose, wrist cam off, dashboard teleop/record/servo "
          f"stopped, nothing else on COM8, you at the arm with a hand free, this window focused (Ctrl-C = E-STOP, "
          f"holds in place).")
    return (ask_tty("Type MOVE to go (anything else cancels): ") or "").strip().upper() == "MOVE"


def server_busy(url: str):
    try:
        link = sweep.ServerLink(url, "")
        for p in ("/teleop/status", "/training/record/status", "/splat/sweep/status"):
            if link.get_json(p).get("running"):
                return f"{p} reports running"
        if link.get_json("/servo/status").get("port") == "COM8":
            return "the dashboard servo bus owns COM8"
    except Exception as exc:  # noqa: BLE001
        return f"dashboard unreachable ({exc!r}); pass --no-server-check if it is not running"
    return None


def connect(a):
    """Real connect through sweep.connect_real (robust_teleop retry patch, calibration write if needed)."""
    if not a.no_server_check:
        why = server_busy(a.server)
        if why:
            print("refusing:", why)
            return None
    try:
        robot = sweep.connect_real(a.port, a.id, max_relative_target=sweep.PARK_LEASH_DEG)
    except Exception:  # noqa: BLE001
        print("connect FAILED - the arm may be UNPOWERED / LIMP. Support it before anything else.\n" + traceback.format_exc())
        return None
    return robot


def start_ok(robot, a) -> bool:
    obs = robot.get_observation()
    bad = sweep.check_rest_pose(sweep.home_from_obs(obs))
    if bad and not a.not_at_rest:
        for b in bad:
            print("start pose:", b)
        print("arm is not at the rest pose: torque is HELD where it is (critic S1). Support the arm, then answer.")
        a = (ask_tty("Type LIMP to release torque (arm supported), anything else = exit holding: ") or "").upper()
        if a.strip() == "LIMP":
            robot.disconnect()
        else:
            robot.bus.disconnect(disable_torque=False)
            print(HELD_WARNING)
        return False
    return True


def fake_for(kind: str, a):
    import fakebus as FB
    if kind == "twin":
        arm = FB.SimArm(seed=a.seed, start=[sweep.TRANSIT[j] for j in J])
        return FB.FakeRobot(arm), arm
    rob = FB.kinematic_robot()
    return rob, rob.arm


def session(a, robot, job, max_runtime_s: float, ask, clock=None, sleep=None):
    jobs.LOGS.mkdir(exist_ok=True)
    log = str(jobs.LOGS / f"{a.cmd}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    cfg = R.RealConfig(speed=a.speed, ack_protection=a.ack_protection)
    code, tc = jobs.run_session(robot, job, cfg, ask=ask, clock=clock, sleep=sleep, max_runtime_s=max_runtime_s,
                                log_path=log)
    if code == sweep.EXIT_NEEDS_ATTENTION and not (tc is not None and tc.limp):
        print(HELD_WARNING)
    print(f"\nexit {code} ({ {0: 'ok', 1: 'error, parked', 4: 'NEEDS ATTENTION: torque HELD', 7: 'watchdog, parked'}.get(code, '') }); log {log}")
    return code, tc


def scripted(answers):
    it = iter(answers)
    return lambda prompt: (print(prompt + "[dry-run]"), next(it, ""))[1]


# ---------------------------------------------------------------------------------------------------- commands
def cmd_check(a) -> int:
    probs = []
    cp = calib_path(a.id)
    print(f"calibration {cp}: {'present' if cp.exists() else 'MISSING'}; fingerprint {fingerprint(a.id)}")
    if not cp.exists():
        probs.append(f"{cp.name} not created yet: gripcal.py measure, then gripcal.py create --entry-tick N")
    elif a.id != "follower_arm" and fingerprint("follower_arm") != fingerprint(a.id):
        probs.append(f"{a.id} arm joints differ from follower_arm (re-teach or fix gripcal)")
    if cp.exists():
        cal = json.loads(cp.read_text())["gripper"]
        bp = R.BladeCal(cal["range_min"], cal["range_max"], cal.get("drive_mode", 0)).problems_for_swap()
        probs += [f"gripper: {p}" for p in bp]
    try:
        docks, fric = load_docks(a.id, need=False)
    except SR.SwapError as exc:
        docks, fric = {}, SR.FrictionCal()
        probs.append(str(exc))
    print(f"docks: {list(docks) or 'none taught'}; friction band {fric.band_deg}")
    if docks:
        name = next(iter(docks))
        p2, info = jobs.plan_checks(docks, REST, [("drop", name), ("pick", name)], True)
        probs += p2
        print("kinematics:", json.dumps(info))
    sv = jobs.service_pose()
    p3 = R.check_clearance(jobs.legs_path(REST, jobs.approach_legs(REST, sv)), True, REST,
                           [d.q_seat for d in docks.values()], "service approach")
    probs += p3
    fq, p4 = jobs.friction_pose(docks, REST)
    probs += p4
    print(f"service pose {np.round(sv, 1).tolist()}; friction pose {np.round(fq, 1).tolist()}")
    if a.real:
        print("check --real connects to the arm: torque goes off for a moment while lerobot configures and is "
              "released at the end. The arm must rest on its stops.")
        if (ask_tty("Type READ to continue: ") or "").strip().upper() != "READ":
            return 1
        robot = connect(a)
        if robot is None:
            return sweep.EXIT_CONNECT
        try:
            rep = tool_session.protection_report(robot.bus)
            tl = robot.bus.sync_read("Torque_Limit", normalize=False)
            temp = robot.bus.sync_read("Present_Temperature", normalize=False)
            print("roll protection registers:", rep)
            print("verdict:", R.protection_verdict(rep, 0.85, 1.8 / a.speed))
            print("Torque_Limit:", tl, " temperatures:", temp)
            print("rest pose:", sweep.check_rest_pose(sweep.home_from_obs(robot.get_observation())) or "ok")
        finally:
            robot.disconnect()                       # connect enabled torque at rest; release (normal end state)
    for p in probs:
        print("PROBLEM:", p)
    print("check:", "ok" if not probs else f"{len(probs)} problem(s)")
    return 0 if not probs else sweep.EXIT_PLAN


def _run_motion(a, kind: str, job_fn, max_s: float, dry_answers=()) -> int:
    if a.dry_run:
        robot, arm = fake_for(kind, a)
        code, tc = session(a, robot, job_fn, max_s, scripted(dry_answers), clock=robot.clock, sleep=robot.sleep)
        return code
    if not confirm_move(a.cmd):
        return 1
    import capguard; capguard.require(f"run_swap.py {a.cmd}")   # 10/29: real motion only on a live take
    robot = connect(a)
    if robot is None:
        return sweep.EXIT_CONNECT
    if not start_ok(robot, a):
        return sweep.EXIT_REST_POSE
    code, tc = session(a, robot, job_fn, max_s, ask_tty)
    return code


def cmd_twist(a) -> int:
    levels = [float(x) / 100.0 for x in a.levels.split(",")]
    out = {}
    code = _run_motion(a, "kin", jobs.job_twist(levels, a.tries, out), 1800.0)
    p = jobs.save_log("twist", out)
    for t in out.get("twists", []):
        print(f"{t['kind']:6s} {t['level']:.0%}: peak {t['peak_load']:.0%} = {t['peak_Nm']} N.m, short {t['short_deg']} "
              f"deg, rest err {t['rest_err_deg']} deg, saturated {t['sat_s']} s, {t['temp_c']} C")
    print("saved", p)
    return code


def cmd_friction(a) -> int:
    docks, _ = load_docks(a.id, need=False)
    q, probs = jobs.friction_pose(docks, REST)
    if probs:
        print("no clear friction pose:", probs)
        return sweep.EXIT_PLAN
    out = {}
    code = _run_motion(a, "kin", jobs.job_friction(q, a.runs, out), 600.0)
    p = jobs.save_log("friction", out)
    print("friction:", json.dumps(out))
    if code == 0 and "band_deg" in out and not a.dry_run:
        b = np.array(out["band_deg"][:4])
        if np.any(b < 0.02) or np.any(b > 4.0):
            print("bands out of the plausible 0.02..4 deg range: NOT saved")
            return 2
        docks_all, _ = load_docks(a.id, need=False)
        SR.save(DOCKS, docks_all, SR.FrictionCal(band_deg=out["band_deg"]), calibration=fingerprint(a.id))
        print(f"saved into {DOCKS} (fingerprint {fingerprint(a.id)}); agreement within 30 %: {out.get('agree_30pct')}")
    print("log", p)
    return code


def cmd_teach(a) -> int:
    out = {}
    code = _run_motion(a, "kin", jobs.job_teach(a.dock, a.n, out), 3600.0, dry_answers=["LIMP"] + [""] * (a.n + 1))
    p = jobs.save_log("teach", out)
    if "q_seat" not in out or out.get("cancelled"):
        print("nothing saved", out)
        return code or 1
    spread = max(out["spread_deg"][:4])
    print(f"q_seat {out['q_seat']}, spread {out['spread_deg']} (pass <= 0.3 deg per joint)")
    if spread > 1.0 and not a.accept_spread:
        print("spread > 1 deg: probably a mis-seat. NOT saved (re-teach, or --accept-spread)")
        return 2
    if not -150.0 <= out["q_seat"][4] <= 100.0:
        print(f"lock roll {out['q_seat'][4]:.1f} deg leaves no room for the +49 deg twist: re-clock the hub. NOT saved")
        return 2
    if a.dry_run:
        print("dry run: not saved", p)
        return code
    docks, fric = load_docks(a.id, need=False)
    docks[a.dock] = SR.DockTeach(a.dock, q_seat=out["q_seat"], blade_park=0.0)
    SR.save(DOCKS, docks, fric, calibration=fingerprint(a.id))
    print(f"saved dock '{a.dock}' into {DOCKS} (fingerprint {fingerprint(a.id)})")
    return code


def cmd_swap(a) -> int:
    if a.cycle:
        actions = [("drop", a.dock), ("pick", a.dock)] * a.cycle
    elif a.drop:
        actions = [("drop", a.drop)]
    elif a.pick:
        actions = [("pick", a.pick)]
    else:
        print("give --drop NAME, --pick NAME or --cycle N --dock NAME")
        return 2
    tool_on = actions[0][0] == "drop"
    if a.dry_run:
        import fakebus as FB
        robot, arm = fake_for("twin", a)
        docks = {n: SR.DockTeach(n, q_seat=arm.q_seat()) for n in {x[1] for x in actions}}
        fric = SR.FrictionCal()
        if not tool_on:
            print("dry run pick-first: the twin starts with the tool on the master; use --drop / --cycle")
            return 2
    else:
        docks, fric = load_docks(a.id)
        missing = [n for _, n in actions if n not in docks]
        if missing:
            print("unknown dock(s):", missing)
            return 2
    probs, info = jobs.plan_checks(docks, R.deg5(robot.get_observation()) if a.dry_run else REST, actions, tool_on)
    if probs:
        for p in probs:
            print("PROBLEM:", p)
        print("plan rejected before any motion")
        return sweep.EXIT_PLAN
    out = {}
    lines = []
    job = jobs.job_swap(actions, docks, fric, tool_on, out, on_result=lambda r: lines.append(r))
    max_s = (60.0 + 60.0 * len(actions)) / a.speed
    if a.dry_run:
        code, tc = session(a, robot, job, max_s, scripted([]), clock=robot.clock, sleep=robot.sleep)
        print("twin truth:", arm.tool_state())
    else:
        if not confirm_move(f"{len(actions)} swap action(s) at speed {a.speed}: " + ", ".join(f"{x} {n}" for x, n in actions)):
            return 1
        import capguard; capguard.require("run_swap.py swap")       # 10/29: real motion only on a live take
        robot = connect(a)
        if robot is None:
            return sweep.EXIT_CONNECT
        if not start_ok(robot, a):
            return sweep.EXIT_REST_POSE
        code, tc = session(a, robot, job, max_s, ask_tty)
    p = jobs.save_log("swap", dict(out, actions=actions, speed=a.speed, exit=code))
    ok = sum(1 for r in out.get("results", []) if r.get("ok"))
    print(f"{ok}/{len(actions)} actions ok; log {p}")
    return code


def cmd_stop(a) -> int:
    R.STOP_FILE.write_text("stop\n")
    print(f"STOP sent ({R.STOP_FILE}): the running changer session holds torque where it is")
    return 0


def cmd_stopbutton(a) -> int:
    import tkinter as tk
    root = tk.Tk()
    root.title("CHANGER STOP")
    root.attributes("-topmost", True)
    root.geometry("380x230+120+120")
    lab = tk.StringVar(value="click / SPACE = STOP\n(the arm HOLDS where it is)")

    def stop(*_):
        R.STOP_FILE.write_text("stop\n")
        lab.set("STOP sent: arm holding.\nAnswer PARK / LIMP / ENTER in the run window")
        btn.configure(bg="#7a0000")
    btn = tk.Button(root, text="STOP", font=("Segoe UI", 44, "bold"), fg="white", bg="#c40000", command=stop)
    btn.pack(fill="both", expand=True)
    tk.Label(root, textvariable=lab, font=("Segoe UI", 11)).pack(fill="x")
    root.bind("<space>", stop)
    root.bind("<Escape>", stop)
    root.mainloop()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--id", default="follower_arm_tc", help="calibration id (gripcal.py makes follower_arm_tc)")
    ap.add_argument("--server", default="http://127.0.0.1:8000")
    ap.add_argument("--no-server-check", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="fake bus, no serial port")
    ap.add_argument("--speed", type=float, default=0.5, choices=[0.5, 1.0],
                    help="0.5 = every move 2x slower (first runs) or 1.0; the only two the twin validated")
    ap.add_argument("--ack-protection", action="store_true")
    ap.add_argument("--not-at-rest", action="store_true")
    ap.add_argument("--seed", type=int, default=5000, help="dry-run twin error-budget seed")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("check")
    c.add_argument("--real", action="store_true")
    t = sub.add_parser("twist")
    t.add_argument("--levels", default="45,55,65,75,85")
    t.add_argument("--tries", type=int, default=3)
    f = sub.add_parser("friction")
    f.add_argument("--runs", type=int, default=2)
    te = sub.add_parser("teach")
    te.add_argument("--dock", required=True)
    te.add_argument("--n", type=int, default=3)
    te.add_argument("--accept-spread", action="store_true")
    s = sub.add_parser("swap")
    s.add_argument("--drop")
    s.add_argument("--pick")
    s.add_argument("--cycle", type=int, default=0)
    s.add_argument("--dock")
    sub.add_parser("stop")
    sub.add_parser("stopbutton")
    a = ap.parse_args(argv)
    if not 0.1 <= a.speed <= 1.0:
        print("--speed must be 0.1..1.0")
        return 2
    return {"check": cmd_check, "twist": cmd_twist, "friction": cmd_friction, "teach": cmd_teach, "swap": cmd_swap,
            "stop": cmd_stop, "stopbutton": cmd_stopbutton}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())

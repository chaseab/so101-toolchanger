# SPDX-License-Identifier: GPL-3.0-or-later
"""End-to-end RealIO tests on the MuJoCo twin (fakebus.SimArm): the real control stack (sweep.run -> TcSweep -> RealIO
-> RealSwapper -> lerobot-like bus) against independent physics, with injected faults.  No serial port.
    cd toolchanger/swap;  set TC_SEAT_R_MM=310;  python -m unittest test_real_io_twin -v
Each test is one drop (+ pick) at speed 1.0 (~30-60 s wall).  Results of the nominal runs: sim/out/realio_twin.jsonl."""
import json
import os
import time
import unittest

os.environ.setdefault("TC_SEAT_R_MM", "310")

import numpy as np  # noqa: E402

import fakebus as FB  # noqa: E402
import jobs  # noqa: E402
import real_io as R  # noqa: E402
import swap_routine as SR  # noqa: E402
from real_io import J, sweep  # noqa: E402

STOP = R.HERE / "STOP_TEST_TWIN"
OUT = R.HERE.parent / "sim" / "out" / "realio_twin.jsonl"


def twin_run(actions=(("drop", "post1"), ("pick", "post1")), fault=None, ask=None, speed=1.0, seed=5000, raw_load=False,
             calibrate=False):
    arm = FB.SimArm(seed=seed, start=[sweep.TRANSIT[j] for j in J])
    fric = SR.FrictionCal()
    if calibrate:      # stand-in for `run_swap.py friction` (exercised on the kinematic fake): the routine's own
        q0 = arm.io.read_q()                       # calibration at the start pose, as the routine MC does
        fric = SR.calibrate_friction(arm.io, q0, hz=50.0, joints=range(4))
    rob = FB.FakeRobot(arm)
    rob.bus.raw_load = raw_load
    docks = {"post1": SR.DockTeach("post1", q_seat=arm.q_seat())}
    holder, st = {}, {}
    if fault is not None:
        arm.hooks.append(lambda a: fault(a, rob, holder, st))
    out = {}
    try:
        STOP.unlink()
    except FileNotFoundError:
        pass
    t0 = time.time()
    code, tc = jobs.run_session(rob, jobs.job_swap(list(actions), docks, fric, True, out),
                                R.RealConfig(speed=speed), ask=ask or (lambda p: ""), clock=rob.clock, sleep=rob.sleep,
                                stop=R.StopFlag(STOP), on_tc=lambda t: holder.setdefault("tc", t), max_runtime_s=600)
    try:
        STOP.unlink()
    except FileNotFoundError:
        pass
    return dict(code=code, tc=tc, arm=arm, rob=rob, st=st, out=out, wall=time.time() - t0)


def mark(st, arm, rob, holder, **kw):
    st.update(t=arm.t, q=dict(arm.positions()), engaged=holder["tc"].engaged, kind=holder["tc"].kind, **kw)


def assert_held(tc_case, r, tol=3.0, need_hold_read=True):
    """Hold-not-park: exit 4, torque still on everywhere, no torque-off disconnect, no P 32 after the fault, body
    joints within tol of where they were when the fault hit (a park moves them tens of degrees)."""
    st, rob, arm = r["st"], r["rob"], r["arm"]
    tc_case.assertIn("t", st, "fault never fired")
    tc_case.assertEqual(r["code"], sweep.EXIT_NEEDS_ATTENTION)
    tc_case.assertTrue(all(rob.bus.torque_on(m) for m in FB.ALL))
    tc_case.assertFalse([o for o in rob.bus.ops if o[1] == "disconnect" and o[3]["disable_torque"]])
    p32 = [v for t, op, reg, v in rob.bus.ops if reg == "P_Coefficient" and t >= st["t"] and 32 in v.values()]
    tc_case.assertEqual(p32, [])
    ref = st["q"] if r["tc"].hold_q is None else {j: float(r["tc"].hold_q[i]) for i, j in enumerate(J)}
    moved = max(abs(arm.positions()[j] - ref[j]) for j in J[:4])      # from where the HOLD started (detection)
    tc_case.assertLess(moved, tol, f"body moved {moved:.1f} deg after the hold started")
    if need_hold_read:                       # with a dead bus the hold cannot read: the servos keep their last goal
        tc_case.assertIsNotNone(r["tc"].hold_q, "no hold happened")


class Kinematics(unittest.TestCase):
    def test_face_offset_matches_sim(self):
        import sim_io as SI
        arm = FB.SimArm(seed=5000, start=None)
        qs_mj = arm.q_seat_mj
        kin = R.FaceKinematics()
        qs = arm.q_seat()
        for w in jobs.routine_waypoints().values():
            q_sim = arm.mj_to_lerobot(arm.io.face_offset_q(qs_mj, *w, qs_mj))
            q_real = kin.face_offset_q(qs, *w, qs)
            if abs(w[0]) < 35 and abs(w[1]) < 40:          # the sim's direct solve from the seat is good only nearby
                self.assertLess(np.max(np.abs(q_sim - q_real)), 0.05, w)


class Twin(unittest.TestCase):
    def test_a_nominal_raw_load(self):
        r = twin_run(raw_load=True)                        # undecoded Present_Load sign bits: no false trips
        self.assertEqual(r["code"], 0, r["tc"].events[-6:])
        res = r["out"]["results"]
        self.assertTrue(all(x["ok"] for x in res))
        rel = r["arm"].tool_state()["rel"]
        self.assertLess(abs(rel["dz"]), 0.6)
        self.assertLess(rel["lat"], 0.8)
        with OUT.open("a") as f:
            f.write(json.dumps({"test": "nominal", "speed": 1.0, "code": r["code"], "results": res, "truth": rel,
                                "peaks": r["tc"].peaks, "max_step": r["out"].get("max_step_deg"),
                                "dilated_s": r["out"].get("dilated_s"), "wall": r["wall"]}, default=float) + "\n")

    def test_b_stall_in_twist_holds(self):
        def f(a, rob, h, st):
            tc = h.get("tc")
            if tc and "t" not in st and tc.roll_tl == 850 and tc.engaged:
                mark(st, a, rob, h)
                a.jam_joint("wrist_roll", 6.0)             # > the 2.5 N.m cap: the roll cannot turn
        r = twin_run(actions=(("drop", "post1"),), fault=f)
        assert_held(self, r)
        self.assertTrue(r["st"]["engaged"])
        self.assertEqual(r["rob"].bus.regs["Torque_Limit"]["wrist_roll"], 250)
        self.assertEqual(r["rob"].bus.regs["Torque_Limit"]["gripper"], 85)

    def test_c_comms_dropout_in_contact_holds(self):
        def f(a, rob, h, st):
            tc = h.get("tc")
            if tc and tc.engaged and "t0" not in st:
                st["t0"] = a.t
            if "t0" in st and "t" not in st and a.t > st["t0"] + 0.3:
                mark(st, a, rob, h)
                rob.bus.fail_when(lambda op, reg: True)
        r = twin_run(actions=(("drop", "post1"),), fault=f)
        assert_held(self, r, need_hold_read=False)
        self.assertTrue(r["st"]["engaged"])

    def test_d_position_error_in_contact_holds(self):
        def f(a, rob, h, st):
            tc = h.get("tc")
            if tc and tc.engaged and tc.kind == "contact" and "t" not in st:
                mark(st, a, rob, h)
                a.jam_joint("shoulder_lift", 12.0)          # the arm is blocked mid-insertion
        r = twin_run(actions=(("drop", "post1"),), fault=f)
        assert_held(self, r)
        self.assertTrue(any("WATCHDOG" in e or "following" in e for e in r["tc"].events), r["tc"].events[-5:])

    def test_e_estop_in_contact_park_refused(self):
        def f(a, rob, h, st):
            tc = h.get("tc")
            if tc and "t" not in st and tc.roll_tl == 850 and tc.engaged:
                mark(st, a, rob, h)
                STOP.write_text("stop")
        r = twin_run(actions=(("drop", "post1"),), fault=f, ask=lambda p: "PARK")
        assert_held(self, r, tol=4.0)
        self.assertTrue(any("PARK refused" in e for e in r["tc"].events))

    def test_f_estop_outside_contact_then_park(self):
        def f(a, rob, h, st):
            tc = h.get("tc")
            if not tc:
                return
            if tc.engaged:
                st["was"] = True
            if st.get("was") and not tc.engaged and "t" not in st:
                st["t1"] = st.get("t1", a.t)
                if a.t > st["t1"] + 1.0:                    # backing away from the parked tool, not engaged
                    mark(st, a, rob, h)
                    STOP.write_text("stop")
        r = twin_run(actions=(("drop", "post1"),), fault=f, ask=lambda p: "PARK" if "PARK" in p else "")
        self.assertIn("t", r["st"])
        self.assertFalse(r["st"]["engaged"])
        self.assertNotEqual(r["code"], sweep.EXIT_NEEDS_ATTENTION, r["tc"].events[-6:])
        home = [sweep.TRANSIT[j] for j in J]
        self.assertLess(max(abs(r["arm"].positions()[j] - home[i]) for i, j in enumerate(J[:4])), sweep.PARK_TOL_DEG)
        self.assertFalse(any(r["rob"].bus.torque_on(m) for m in FB.ALL))          # released at home
        d = r["arm"].tool_state()["dock"]
        self.assertGreater(d["z"], -5.0)                    # the parked tool was not dragged off the dock
        self.assertLess(d["tilt"], 5.0)

    def test_g_gripper_reboot_while_engaged(self):
        def f(a, rob, h, st):
            tc = h.get("tc")
            if tc and tc.engaged and "t" not in st:
                mark(st, a, rob, h)
                rob.bus.reboot("gripper")
        r = twin_run(actions=(("drop", "post1"),), fault=f)
        assert_held(self, r, tol=4.0)
        self.assertEqual(r["rob"].bus.regs["Torque_Limit"]["gripper"], 85)        # re-capped by the hold


if __name__ == "__main__":
    unittest.main()

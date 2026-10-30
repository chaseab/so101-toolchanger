# SPDX-License-Identifier: GPL-3.0-or-later
"""Fast offline tests for real_io / jobs / gripcal on the kinematic fake bus (no serial port, no MuJoCo).
    cd toolchanger/swap;  python -m unittest test_real_io -v
The MuJoCo end-to-end fault tests are in test_real_io_twin.py."""
import json
import math
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

import fakebus as FB
import gripcal
import jobs
import real_io as R
import swap_routine as SR
import tool_session
from real_io import J, sweep

STOP = R.HERE / "STOP_TEST_FAST"


def session(robot, job, ask=None, speed=1.0, on_tc=None, max_s=600.0):
    stop = R.StopFlag(STOP)
    try:
        STOP.unlink()
    except FileNotFoundError:
        pass
    return jobs.run_session(robot, job, R.RealConfig(speed=speed), ask=ask or (lambda p: ""), clock=robot.clock,
                            sleep=robot.sleep, stop=stop, on_tc=on_tc, max_runtime_s=max_s)


def writes(bus, reg, after=0.0):
    return [(t, v) for t, op, r, v in bus.ops if op == "write" and r == reg and t >= after]


class Pure(unittest.TestCase):
    def test_decode_load(self):
        self.assertEqual(R.decode_load(-20), -20)          # lerobot-decoded negative stays (camarm 10/04 bug)
        self.assertEqual(R.decode_load(20), 20)
        self.assertEqual(R.decode_load(0x400 | 20), -20)   # raw sign-magnitude
        self.assertEqual(R.decode_load(1000), 1000)

    def test_limiter_no_overshoot(self):
        L = R.Limiter([25.0], [100.0], 0.04)
        L.reset([0.0])
        xs = [float(L.step([10.0])[0]) for _ in range(200)]
        self.assertLessEqual(max(xs), 10.0 + 1e-9)
        self.assertAlmostEqual(xs[-1], 10.0)
        steps = np.diff([0.0] + xs)
        self.assertLessEqual(steps.max(), 25.0 * 0.04 + 1e-9)
        self.assertLessEqual(np.diff(steps).max(), 100.0 * 0.04 ** 2 + 1e-9)

    def test_blade_cal(self):
        wide = R.BladeCal(57, 4039)
        self.assertEqual(wide.problems_for_swap(), [])
        for d in (10.0, 150.0, 340.0):
            self.assertAlmostEqual(wide.deg(wide.pct(d)), d, places=6)
        stock = R.BladeCal(1270, 2667)
        self.assertTrue(any("stock" in p for p in stock.problems_for_swap()))

    def test_protection_verdict(self):
        rep = dict(Max_Torque_Limit=1000, Overload_Torque=80, Protection_Time=200, Protective_Torque=20)
        block, notes = R.protection_verdict(rep, 0.85, 3.6)
        self.assertEqual(block, [])
        self.assertTrue(notes)
        block, _ = R.protection_verdict(dict(rep, Max_Torque_Limit=500), 0.85, 1.8)
        self.assertTrue(block)

    def test_bridge_and_waypoints(self):
        qs = [0.0, 17.9, 48.5, -38.3, -4.1]                 # a 310 mm seat (twin) in lerobot degrees
        kin = R.FaceKinematics()
        self.assertLess(tool_session.check_kinematics_bridge(kin.face_offset_q, qs), 0.5)
        for name, w in jobs.routine_waypoints().items():
            q = kin.face_offset_q(qs, *w, qs)
            c = kin.dock_coords(qs, q)
            self.assertLess(math.hypot(c[0] - w[0], c[2] - w[1]), 0.5, name)
            self.assertLess(abs(c[1]), 0.5, name)

    def test_zone(self):
        self.assertTrue(R.in_zone((0, 3, 0)))
        self.assertTrue(R.in_zone((1.7, 0, 45)))
        self.assertFalse(R.in_zone((0, 35, 0)))             # A
        self.assertFalse(R.in_zone((-30, 0, 45)))           # E
        self.assertFalse(R.in_zone((-60, 45, 0)))           # F

    def test_gripcal(self):
        stock = json.loads(json.dumps(FB.DEFAULT_CAL))
        stock["gripper"] = dict(FB.STOCK_GRIPPER)
        tc = gripcal.make_tc(stock, 2100)
        for j in J:
            self.assertEqual(tc[j], stock[j])
        g = tc["gripper"]
        self.assertEqual((g["range_min"], g["range_max"]), (gripcal.TC_MIN, gripcal.TC_MAX))
        self.assertEqual((2100 + stock["gripper"]["homing_offset"] - g["homing_offset"]) % 4096, gripcal.ENTRY_TICK)
        self.assertLessEqual(abs(g["homing_offset"]), 2047)
        self.assertEqual([p for p in gripcal.problems(tc, stock) if "fingerprint" not in p], [])

    def test_dock_file_guards(self):
        with tempfile.TemporaryDirectory() as t:
            p = os.path.join(t, "d.json")
            SR.save(p, {"a": SR.DockTeach("a", [0, 0, 0, 0, -4.0])}, SR.FrictionCal(), "fp1")
            with self.assertRaises(SR.SwapError):
                SR.load(p, "fp2")


class Sessions(unittest.TestCase):
    def test_twist_ok_parks_and_restores(self):
        rob = FB.kinematic_robot()
        out = {}
        code, tc = session(rob, jobs.job_twist([0.45, 0.85], 1, out))
        self.assertEqual(code, 0)
        self.assertEqual(len(out["twists"]), 4)
        self.assertEqual(rob.bus.regs["Torque_Limit"]["wrist_roll"], 1000)          # restored at the end
        self.assertEqual(rob.bus.regs["Torque_Limit"]["gripper"], 85)               # session cap all along
        self.assertFalse(any(rob.bus.torque_on(m) for m in FB.ALL))                 # released at home (rest)
        self.assertLess(max(abs(rob.arm.pos[j] - sweep.REST_EXPECTED[j]) for j in J[:4]), sweep.PARK_TOL_DEG)
        caps = [v for _, v in writes(rob.bus, "Torque_Limit") if "gripper" in v]
        self.assertTrue(all(v["gripper"] <= 85 for v in caps))

    def test_estop_outside_contact_holds(self):
        rob = FB.kinematic_robot()
        fired = {}

        def hook(arm):
            if "t" not in fired and rob.bus.regs["Torque_Limit"]["wrist_roll"] == 450:
                fired["t"] = arm.t
                fired["q"] = dict(arm.pos)
                STOP.write_text("stop")
        rob.arm.hooks.append(hook)
        code, tc = session(rob, jobs.job_twist([0.45], 1, {}))
        self.assertEqual(code, sweep.EXIT_NEEDS_ATTENTION)
        self.assertTrue(all(rob.bus.torque_on(m) for m in FB.ALL))                  # torque held
        self.assertEqual(writes(rob.bus, "P_Coefficient", fired["t"]) and
                         [v for _, v in writes(rob.bus, "P_Coefficient", fired["t"]) if 32 in v.values()], [])
        moved = max(abs(rob.arm.pos[j] - fired["q"][j]) for j in J[:4])
        self.assertLess(moved, 3.0)                                                 # no park motion
        self.assertEqual(rob.bus.regs["Torque_Limit"]["wrist_roll"], 250)
        self.assertFalse([o for o in rob.bus.ops if o[1] == "disconnect" and o[3]["disable_torque"]])

    def test_estop_then_operator_park(self):
        rob = FB.kinematic_robot()
        rob.arm.hooks.append(lambda arm: STOP.write_text("stop") if rob.bus.regs["Torque_Limit"]["wrist_roll"] == 450
                             and not STOP.exists() and arm.t < 1e9 and not getattr(arm, "_done", False)
                             and setattr(arm, "_done", True) is None else None)
        code, tc = session(rob, jobs.job_twist([0.45], 1, {}), ask=lambda p: "PARK" if "PARK" in p else "")
        self.assertNotEqual(code, sweep.EXIT_NEEDS_ATTENTION)
        self.assertLess(max(abs(rob.arm.pos[j] - sweep.REST_EXPECTED[j]) for j in J[:4]), sweep.PARK_TOL_DEG)
        self.assertFalse(any(rob.bus.torque_on(m) for m in FB.ALL))

    def test_ctrl_c_flag(self):
        rob = FB.kinematic_robot()
        holder = {}
        rob.arm.hooks.append(lambda arm: setattr(holder["tc"].stop, "sigint", True)
                             if "tc" in holder and rob.bus.regs["Torque_Limit"]["wrist_roll"] == 450 else None)
        code, tc = session(rob, jobs.job_twist([0.45], 1, {}), on_tc=lambda t: holder.setdefault("tc", t))
        self.assertEqual(code, sweep.EXIT_NEEDS_ATTENTION)
        self.assertIn("Ctrl-C", tc.fault_reason)

    def test_comms_dropout_holds(self):
        rob = FB.kinematic_robot()
        st = {}

        def hook(arm):
            if "t" not in st and rob.bus.regs["Torque_Limit"]["wrist_roll"] == 450:
                st["t"] = arm.t
                rob.bus.fail_when(lambda op, reg: True)
        rob.arm.hooks.append(hook)
        code, tc = session(rob, jobs.job_twist([0.45], 1, {}))
        self.assertEqual(code, sweep.EXIT_NEEDS_ATTENTION)                          # park fails -> hold
        self.assertTrue(all(rob.bus.torque_on(m) for m in FB.ALL))

    def test_gripper_reboot_detected(self):
        rob = FB.kinematic_robot()
        rob.arm.hooks.append(lambda arm: rob.bus.reboot("gripper") if 4.0 < arm.t < 4.01 else None)
        code, tc = session(rob, jobs.job_twist([0.45], 2, {}))
        self.assertNotEqual(code, 0)
        self.assertTrue(any("gripper cap lost" in e for e in tc.events))
        self.assertEqual(rob.bus.regs["Torque_Limit"]["gripper"], 85)               # re-capped

    def test_teach_limp_body_only(self):
        rob = FB.kinematic_robot()
        out = {}
        answers = iter(["LIMP", "", "", "", ""])
        code, tc = session(rob, jobs.job_teach("post1", 3, out), ask=lambda p: next(answers, ""))
        self.assertEqual(code, 0)
        self.assertEqual(len(out["samples"]), 3)
        tl = [v for _, v in writes(rob.bus, "Torque_Enable") if "gripper" in v and v["gripper"] == 0]
        body_off = [v for _, v in writes(rob.bus, "Torque_Enable") if "shoulder_lift" in v and v["shoulder_lift"] == 0]
        self.assertTrue(body_off)
        self.assertEqual(len(tl), 1)                       # the gripper only goes limp at the final disconnect
        self.assertFalse([1 for t, o in rob.sent])         # no Goal_Position stream while limp... (none at all here)

    def test_teach_cancel_no_limp(self):
        rob = FB.kinematic_robot()
        out = {}
        code, tc = session(rob, jobs.job_teach("post1", 3, out), ask=lambda p: "no")
        self.assertTrue(out.get("cancelled"))
        self.assertFalse([v for _, v in writes(rob.bus, "Torque_Enable") if v.get("shoulder_lift") == 0][:-1])

    def test_protection_block_refuses_before_motion(self):
        rob = FB.kinematic_robot()
        rob.bus.regs["Max_Torque_Limit"]["wrist_roll"] = 500
        code, tc = session(rob, jobs.job_twist([0.85], 1, {}))
        self.assertNotEqual(code, 0)
        self.assertEqual(rob.sent and max(abs(v["shoulder_lift.pos"] - sweep.REST_EXPECTED["shoulder_lift"])
                                          for _, v in rob.sent), 0.0 if rob.sent else [])

    def test_stock_gripper_refused_for_swap(self):
        rob = FB.kinematic_robot(gripper_cal=FB.STOCK_GRIPPER, start={"gripper": 180.0})
        qs = [0.0, 17.9, 48.5, -38.3, -4.1]
        code, tc = session(rob, jobs.job_swap([("drop", "d")], {"d": SR.DockTeach("d", qs)}, SR.FrictionCal(), True, {}))
        self.assertNotEqual(code, 0)
        self.assertFalse(rob.sent)                         # nothing moved

    def test_jump_guard(self):
        rob = FB.kinematic_robot()
        caught = {}

        def job(tc):
            io = R.RealIO(tc)
            io.setup(False)
            q = R.deg5(tc.robot.get_observation())
            io.resync()
            io.set_blade_ref(io.blade.deg(float(tc.robot.get_observation()["gripper.pos"])))
            io.write(q, 0.0)
            try:
                io.write(q + np.array([0, 20.0, 0, 0, 0]), 0.0)
            except SR.SwapError as e:
                caught["e"] = str(e)
        code, tc = session(rob, job)
        self.assertIn("jump", caught.get("e", ""))


if __name__ == "__main__":
    unittest.main()

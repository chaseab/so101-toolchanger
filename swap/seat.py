# SPDX-License-Identifier: GPL-3.0-or-later
"""Error-tolerant insert + blade search + lock for a PICK, from a free standoff in front of the parked tool (10/29).

    python seat.py [--max 14] [--speed 1.5] [--pan-soft 30] [--no-lock]

Start: master free, roll at the entry angle (lock + ~47 deg), tool axis roughly lined up, face 5-30 mm in front.
1. stream along the tool axis (smooth.py's IK plan + integral loop) until CONTACT: the integral push on elbow/lift grows
   > 1.5 deg past its free-motion value or elbow load rises > 10 % over its free baseline (10/29: contact showed as
   load 14 -> 36 % with < 0.5 deg lag, so lag alone is blind to it).
2. hold contact + 1 mm preload and sweep the blade (gripper servo, Torque_Limit cap 0.25 N.m) up to 180 deg, then back;
   the blade dropping into the rotor slot = the face advancing >= 2 mm along the axis (DESIGN_v4: a mis-parked rotor
   stops the insertion ~4 mm short).
3. push on to full depth (second contact), release the preload to ~light, twist roll -47 deg; LOCK = roll peak load
   >= 8 % (the owner's 10/29 teleop lock: 10-18 %; a free spin: 5 %).
Every step logs to logs/seat_<time>.csv. Any watchdog -> HOLD, prints why. Exit code 0 = locked, 2 = no rotor found,
3 = twist spun free, 1 = watchdog.
"""
import csv, os, sys, time
import numpy as np
import real_io as R
from where import d5_from_ticks, CAL, KEYS
from plan import ik, ticks_from_d5
import jog as J

HERE = os.path.dirname(os.path.abspath(__file__))
TPD = 4096 / 360.0
KI = np.array([2.0, 4.0, 3.0, 3.0]); ILIM = np.array([3.0, 12.0, 10.0, 6.0])
a = sys.argv[1:]
def opt(n, d): return float(a[a.index(n) + 1]) if n in a else d
MAXMM, SPEED, PANSOFT = opt("--max", 14.0), opt("--speed", 1.5), opt("--pan-soft", 30)
LOG = open(os.path.join(HERE, "logs", f"seat_{time.strftime('%Y%m%d_%H%M%S')}.csv"), "w", newline="")
W = csv.writer(LOG); W.writerow(["t", "phase", "s_mm", "y_ax_mm", "I_lift", "I_elbow", "load_lift", "load_elbow", "blade", "roll", "roll_load"])
T0 = time.time()


def tk():
    return np.array([J.rd(i, 56, 2) for i in range(1, 7)], float)


def d5(t):
    return np.array(d5_from_ticks(t[:5]))


def along(d, p0, ax):
    p, _ = R.face_pose(list(d)); return float((p - p0) @ ax) * 1000


class Streamer:
    """Holds an IK path along the axis; .go(s_mm) streams the plan point at s with the integral loop."""
    def __init__(self):
        t = tk(); self.d0 = d5(t)
        self.p0, R0 = R.face_pose(list(self.d0)); self.ax = -R0[:, 2]
        n = int(MAXMM) + 2; self.sgrid = np.arange(n, dtype=float)
        seed, pl = list(self.d0), []
        for s in self.sgrid:
            d, e, ae = ik(self.p0 + self.ax * s / 1000, self.ax, seed, iters=400)
            if e > 1 or ae > 1: sys.exit(f"IK miss at {s} mm")
            d[4] = self.d0[4]; pl.append(d[:5]); seed = list(d)
        self.plan = np.array(pl)
        g0 = np.array([J.rd(i, 42, 2) for i in range(1, 5)], float)
        self.I = np.clip((g0 - t[:4]) / TPD, -ILIM, ILIM); self.last = time.time()
        for i in range(1, 5): J.wr(i, 41, 1, 0); J.wr(i, 46, 2, 0)
        if PANSOFT: J.wr(1, 48, 2, int(PANSOFT * 10))

    def plan_at(self, s):
        s = float(np.clip(s, 0, self.sgrid[-1])); i = min(int(s), len(self.sgrid) - 2); f = s - i
        return self.plan[i] * (1 - f) + self.plan[i + 1] * f

    def go(self, s, phase, extra=None):
        now = time.time(); dt = min(0.1, now - self.last); self.last = now
        dp = self.plan_at(s); t = tk(); dm = d5(t)
        e = dp[:4] - dm[:4]; self.I = np.clip(self.I + KI * e * dt, -ILIM, ILIM)
        gt = ticks_from_d5(list(dp[:4] + self.I) + [dp[4]])
        for k in range(4): J.wr(k + 1, 42, 2, gt[k])
        ld = np.array([J.load(k + 1) for k in range(4)])
        y = along(dm, self.p0, self.ax)
        W.writerow([f"{now - T0:.2f}", phase, f"{s:.2f}", f"{y:.2f}", f"{self.I[1]:.2f}", f"{self.I[2]:.2f}", ld[1], ld[2],
                    int(t[5]), int(t[4]), J.load(5)])
        if (np.abs(ld) >= 75).any() or (np.abs(e) > [6, 12, 10, 8]).any():
            raise RuntimeError(f"watchdog: load {ld} err {np.round(e, 1)}")
        return y, ld, e

    def close(self):
        if PANSOFT: J.wr(1, 48, 2, 800)


def advance_to_contact(st, s_from, phase, free_s=2.0):
    """Ramp the plan forward at SPEED until contact. Returns (s_at_contact, y_at_contact) or (None, y)."""
    s = s_from; t_start = time.time(); I0 = None; L0 = []
    while s < MAXMM:
        s = s_from + SPEED * (time.time() - t_start)
        y, ld, e = st.go(s, phase)
        if time.time() - t_start < free_s:                  # baseline: free-motion integral + elbow load
            L0.append(ld[2]); I0 = st.I.copy(); continue
        base = np.median(L0)
        if abs(st.I[2] - I0[2]) > 1.5 or abs(st.I[1] - I0[1]) > 1.5 or abs(ld[2] - base) > 10:
            return s, y
        time.sleep(0.02)
    return None, y


def hold_at(st, s, secs, phase):
    t1 = time.time() + secs
    while time.time() < t1:
        y, ld, e = st.go(s, phase); time.sleep(0.02)
    return y


def blade_search(st, s_hold, y_contact):
    """Sweep the blade +180 then -180 deg from where it is (inside the tc calibration range) at the 0.25 N.m cap."""
    b0 = J.rd(6, 56, 2)
    lo, hi = 60, 4030                                       # follower_arm_tc wide range (57..4039) minus margin
    for direction in (+1, -1):
        for k in range(1, 61):                              # 3 deg per step, 60 steps = 180 deg
            g = int(np.clip(b0 + direction * k * 3 * TPD, lo, hi))
            J.wr(6, 42, 2, g)
            y = hold_at(st, s_hold, 0.12, f"blade{'+' if direction > 0 else '-'}")
            if y - y_contact >= 2.0:
                return True, g, y
        J.wr(6, 42, 2, b0); hold_at(st, s_hold, 1.0, "blade-return")
    return False, b0, y


def main():
    import capguard; capguard.require("seat.py")
    J.start_trace("seat")
    st = Streamer(); code = 1
    try:
        s_c, y_c = advance_to_contact(st, 0.0, "approach")
        if s_c is None:
            print(f"no contact within {MAXMM} mm (face moved {y_c:.1f} mm): tool not in front of the master"); code = 2; return
        print(f"contact at s={s_c:.1f} mm (face {y_c:.1f} mm along the axis)")
        if "--contact-only" in a:
            code = 0; return
        s_hold = s_c + 1.0
        ok, bgoal, y = blade_search(st, s_hold, y_c)
        if not ok:
            print("blade search: the face never advanced 2 mm -> no rotor slot found"); code = 2; return
        print(f"blade dropped in at blade tick {bgoal}: face advanced {y - y_c:.1f} mm")
        s2, y2 = advance_to_contact(st, s_hold, "seat", free_s=0.8)
        s_seat = (s2 if s2 is not None else st.sgrid[-1])
        y = hold_at(st, s_seat + 0.5, 0.8, "seated")
        print(f"seated: face {y:.1f} mm along the axis (contact {y_c:.1f}, blade drop +{y - y_c:.1f})")
        if "--no-lock" in a:
            code = 0; return
        # twist: keep streaming the axial hold while roll moves (jog's slow profile), watch the roll load
        r0 = J.rd(5, 56, 2); J.wr(5, 41, 1, 10); J.wr(5, 46, 2, 120); J.wr(5, 42, 2, int(r0 - 47 * TPD))
        peak = 0.0; t1 = time.time() + 6
        while time.time() < t1:
            st.go(s_seat + 0.5, "twist"); peak = max(peak, abs(J.load(5)))
            if abs(J.rd(5, 56, 2) - (r0 - 47 * TPD)) < 10: break
        hold_at(st, s_seat, 0.5, "locked?")
        rnow = J.rd(5, 56, 2)
        print(f"twist: roll {r0} -> {rnow} ticks ({(rnow - r0) / TPD:+.1f} deg), peak roll load {peak:.0f} %")
        code = 0 if peak >= 8 else 3
        print("LOCKED (roll load)" if code == 0 else "twist spun free: NOT locked")
    except RuntimeError as e:
        print("HOLD:", e); J.hold()
    except KeyboardInterrupt:
        print("Ctrl-C -> HOLD"); J.hold()
    finally:
        J.hold(); st.close(); LOG.close(); print("log", LOG.name)
        sys.exit(code)


if __name__ == "__main__":
    main()

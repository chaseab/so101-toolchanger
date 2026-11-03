# SPDX-License-Identifier: GPL-3.0-or-later
"""NUDGE console (10/31, swap-desk): shared control for the dock pick/drop. operator steers the last mm, the script does
the travel, the holding and the twist, and every key + pose is logged so the moves that work become the replay.

    python nudge.py [--no-capture]       (run in its own terminal; keys below; X quits and HOLDS torque)

Moves are in the BASE frame (the dock slot runs along +y):
    W / S   push IN (+y) / pull OUT (-y)          A / D   left (-x) / right (+x)       R / F   up (+z) / down (-z)
    T / G   tip UP / DOWN (tool-axis z +-0.01 = ~0.6 deg)                              Q / E   twist roll - / +
    1 2 3 4 step size 0.5 / 1 / 2 / 5 mm  (roll 1 / 2 / 5 / 10 deg)                    [ / ]   slower / faster
Auto:
    O   go to the standoff in front of the dock (joint move to a high ready pose, then straight in)
    I   push in slowly until contact (stops on load rise / lag; any key aborts)
    L   twist to STALL (= lock attempt, 85 % roll torque)    U  unlock (roll back to entry)   Z  relax roll load to ~0
    Y   lift straight up 20 mm (slow, pitch held)            K  pan soft on/off (dock steers the tool sideways)
Record:
    P   save this pose as the standoff           M  mark result (type a word: lock / miss / snag / good ...)
    SPACE  freeze / unfreeze                     X  quit (torque stays ON, arm holds where it is)
Safety: every tick checks load >= 75 % for 0.5 s, temp >= 55 C, tracking error, the swap/STOP file -> FREEZE (hold).
Logs: logs/nudge_<time>.csv (10 Hz pose/load) + logs/nudge_events.jsonl (every key, pose before/after).
"""
import collections, csv, json, msvcrt, os, sys, threading, time
import numpy as np
import real_io as R
from where import d5_from_ticks, CAL, KEYS
from plan import ticks_from_d5
import jog as J
import tc_config  # noqa: E402  (machine-specific settings: config.json, see config.example.json)

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "nudge_cfg.json")
EVT = os.path.join(HERE, "logs", "nudge_events.jsonl")
TPD = 4096 / 360.0
_TUNE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ctl_tune.json")   # 11/02 eve: per-joint feed-forward lead
FF_TAU = np.array(json.load(open(_TUNE)).get("ff_tau", [0, 0, 0, 0]), float) if os.path.exists(_TUNE) else np.zeros(4)
FF_MAX = 3.0                                                                          # deg cap on the lead
KI = np.array([2.0, 4.0, 3.0, 3.0]); ILIM = np.array([3.0, 12.0, 8.0, 6.0]); ERRLIM = np.array([6.0, 12.0, 8.0, 8.0])
STEPS = {"1": (0.5, 1), "2": (1.0, 2), "3": (2.0, 5), "4": (5.0, 10)}
ROLL_CAP = 450                                    # 11/02 pm-eng: wrist roll Torque_Limit between moves (was 1000 = brownouts 11/01)
IDLE_STOP = 300                                   # s without a key -> stop the recording (C: space, 10/31)
DEF_CFG = {"standoff": [52.5, 234.0, 103.0, 0.05], "ready": [52.5, 205.0, 115.0, 0.05], "entry": 2487}


def load_cfg():
    c = dict(DEF_CFG)
    if os.path.exists(CFG):
        c.update(json.load(open(CFG)))
    return c


def read5():
    return np.array(d5_from_ticks([J.rd(i, 56, 2) for i in range(1, 6)]))


def pose(d5):
    p, Rm = R.face_pose(list(d5)); ax = -Rm[:, 2]
    return np.concatenate([p * 1000, [ax[2]]])          # x y z (mm), tool-axis z (pitch)


def ik4(target, seed5, iters=30):
    """Pan/lift/elbow/flex for face (x,y,z mm) + tool-axis z; roll held. Newton with a numeric Jacobian (~10 ms)."""
    q = np.array(seed5, float); W = np.array([1, 1, 1, 200.0])     # 0.01 of axis-z ~ 2 mm of position
    for _ in range(iters):
        r = (np.asarray(target) - pose(q)) * W
        if np.abs(r[:3]).max() < 0.05 and abs(r[3]) < 0.2:
            return q, True
        Jm = np.zeros((4, 4))
        for k in range(4):
            dq = q.copy(); dq[k] += 0.05
            Jm[:, k] = (pose(dq) - pose(q)) * W / 0.05
        q[:4] += np.clip(np.linalg.solve(Jm.T @ Jm + 1e-4 * np.eye(4), Jm.T @ r), -5, 5)
    r = (np.asarray(target) - pose(q)) * W
    return q, bool(np.abs(r[:3]).max() < 0.5 and abs(r[3]) < 1.0)


def in_range(q):
    t = ticks_from_d5(list(q))
    return all(CAL[KEYS[k]]["range_min"] <= t[k] <= CAL[KEYS[k]]["range_max"] for k in range(4))


class Ctl:
    """30 Hz servo loop on pan/lift/elbow/flex: goal = plan + I (integral takes up sag). Segments are min-jerk."""
    def __init__(self):
        self.lock = threading.Lock(); self.segs = collections.deque()
        q = read5(); self.plan = q[:4].copy(); self.target_q = q.copy()
        t = np.array([J.rd(i, 56, 2) for i in range(1, 5)], float); g = np.array([J.rd(i, 42, 2) for i in range(1, 5)], float)
        self.I = np.clip((g - t) / TPD, -ILIM, ILIM)
        self.frozen = False; self.why = ""; self.run = True; self.meas = q; self.ld = np.zeros(5); self.temp = 0
        self.seg = None; self.busy = False; self.contact_lag = np.zeros(4); self._pp = None; self._v = np.zeros(4)
        self.csv = open(os.path.join(HERE, "logs", f"nudge_{time.strftime('%Y%m%d_%H%M%S')}.csv"), "w", newline="", buffering=1)
        self.w = csv.writer(self.csv)
        self.w.writerow(["t", *[f"plan_{k}" for k in KEYS[:4]], *[f"meas_{k}" for k in KEYS], *[f"load_{n}" for n in J.NAMES[:5]],
                         "x", "y", "z", "az"])
        self.step_tag = ""
        for i in range(1, 5):
            J.wr(i, 41, 1, 0); J.wr(i, 46, 2, 0)
        self.th = threading.Thread(target=self.loop, daemon=True); self.th.start()

    def add(self, q4, secs, lin=False):
        with self.lock:                                  # lin: constant-rate piece of a longer smooth move (11/01)
            self.segs.append((np.array(q4[:4], float), max(0.03, secs) if lin else max(0.25, secs), lin))

    def idle(self):
        return not self.segs and self.seg is None

    def freeze(self, why):
        with self.lock:
            self.segs.clear(); self.seg = None; self.frozen = True; self.why = why
        J.hold()
        q = read5(); self.plan = q[:4].copy(); self.I[:] = 0; self.target_q[:4] = q[:4]

    def unfreeze(self):
        q = read5(); self.plan = q[:4].copy(); self.I[:] = 0; self.target_q[:4] = q[:4]; self.frozen = False; self.why = ""

    def loop(self):
        last = time.time(); over = np.zeros(4); n = 0
        while self.run:
            now = time.time(); dt = now - last; last = now
            try:
                q = read5(); self.meas = q
                if not self.frozen:
                    with self.lock:
                        if self.seg is None and self.segs:
                            q1, T, lin = self.segs.popleft(); self.seg = (self.plan.copy(), q1, now, T, lin)
                    if self.seg is not None:
                        q0, q1, t0, T, lin = self.seg; tau = min(1.0, (now - t0) / T)
                        with self.lock:                  # 11/02 chain: a finished piece hands straight to the next queued one
                            while tau >= 1.0 and self.segs:   # (was: 1 still tick per 0.1 s piece = 25 % stutter, 1.33x slow)
                                n1, nT, nlin = self.segs.popleft(); self.seg = (q1.copy(), n1, t0 + T, nT, nlin)
                                q0, q1, t0, T, lin = self.seg; tau = min(1.0, (now - t0) / T)
                        s = tau if lin else 10 * tau**3 - 15 * tau**4 + 6 * tau**5
                        self.plan = q0 + (q1 - q0) * s
                        if tau >= 1.0:
                            self.seg = None
                    e = self.plan - q[:4]; self.contact_lag = e
                    ld = np.array([J.load(i) for i in range(1, 6)]); self.ld = ld
                    hi = np.abs(ld[:4]) > 40                 # pressing on something: stop winding the integral up
                    self.I = np.clip(self.I + np.where(hi, 0, KI * e * dt), -ILIM, ILIM)
                    vel = (self.plan - self._pp) / max(dt, 0.01) if self._pp is not None else np.zeros(4)
                    self._pp = self.plan.copy(); self._v = 0.5 * self._v + 0.5 * vel     # 11/02 eve: velocity feed-forward
                    lead = np.clip(FF_TAU * self._v, -FF_MAX, FF_MAX)                     # (ctl_tune.json ff_tau s, 0 = off)
                    gt = ticks_from_d5(list(self.plan + self.I + lead) + [q[4]])
                    for k in range(4):
                        J.wr(k + 1, 42, 2, gt[k])
                    over = np.where(np.abs(ld[:4]) >= 75, over + dt, 0)
                    if (over >= 0.5).any():
                        self.freeze(f"load >= 75 % on {[J.NAMES[k] for k in np.where(over >= 0.5)[0]]}")
                    elif (np.abs(e) > ERRLIM).any():
                        self.freeze(f"tracking error {np.round(e, 1)} deg")
                    elif os.path.exists(os.path.join(HERE, "STOP")):
                        self.freeze("STOP file")
                else:
                    self.ld = np.array([J.load(i) for i in range(1, 6)]); self._pp = None; self._v = np.zeros(4)
                n += 1
                if n % 15 == 0:
                    raw = np.array([J.rd(k, 63, 1) for k in range(1, 6)])      # 11/02 eve: misreads (43->57, 47->56->150) froze n100.
                    conf = getattr(self, "_tconf", raw.copy()); cnt = getattr(self, "_tcnt", np.zeros(5, int))
                    jump = (np.abs(raw - conf) > 5) | (raw > 90)                                # real heating is ~1 C per 30 s cycle: a >5 C jump in
                    cnt = np.where(jump, cnt + 1, 0)                              # one 0.6 s read only counts after 3 reads in a row
                    conf = np.where(~jump | ((cnt >= 3) & (raw <= 90)), raw, conf); self._tconf, self._tcnt = conf, cnt
                    self.temps = [int(x) for x in conf]; self.temps_raw = [int(x) for x in raw]
                    if jump.any() and (cnt < 3).any(): ev("temp_spike", raw=self.temps_raw, confirmed=self.temps)
                    self.temp = int(conf[1:].max())
                    if self.temp >= 55 and not self.frozen:
                        if max(J.rd(k, 63, 1) for k in range(1, 6)) >= 55:      # 3rd read, still hot: freeze
                            self.freeze(f"temp {self.temp} C")
                if n % (1 if getattr(self, 'fast_log', False) else 3) == 0:   # 11/01: full 30 Hz during paths
                    self.w.writerow([f"{now:.3f}", *np.round(self.plan, 2), *np.round(q, 2), *self.ld, *np.round(pose(q), 3)])
            except IOError as ex:
                self.why = f"bus: {ex}"
            time.sleep(max(0, 0.03 - (time.time() - now)))


def ev(kind, **kw):
    with open(EVT, "a") as f:
        f.write(json.dumps({"t": time.strftime("%Y-%m-%d %H:%M:%S"), "kind": kind, **kw}) + "\n")


def roll_goto(goal, speed=150, secs=15):
    J.wr(5, 48, 2, ROLL_CAP); J.wr(5, 41, 1, 10); J.wr(5, 46, 2, speed); J.wr(5, 42, 2, int(goal)); t0 = time.time()
    while time.time() - t0 < secs and abs(J.rd(5, 56, 2) - goal) > 10:
        time.sleep(0.1)
    return J.rd(5, 56, 2)


def twist_to_stall():
    r0 = J.rd(5, 56, 2)
    J.wr(5, 48, 2, 850); J.wr(5, 41, 1, 10); J.wr(5, 46, 2, 60); J.wr(5, 42, 2, int(r0 - 80 * TPD))
    t0, last, still, pk = time.time(), r0, 0, 0
    pel, vmin = abs(J.load(3)), J.rd(2, 62, 1) / 10          # 11/02 pm-eng: log elbow + bus V for every twist
    while time.time() - t0 < 20:
        time.sleep(0.1); p = J.rd(5, 56, 2); pk = max(pk, abs(J.load(5)))
        pel = max(pel, abs(J.load(3))); vmin = min(vmin, J.rd(2, 62, 1) / 10)
        still = still + 1 if abs(p - last) < 2 else 0; last = p
        if still >= 6 or msvcrt.kbhit(): break
    J.wr(5, 42, 2, J.rd(5, 56, 2)); J.wr(5, 48, 2, ROLL_CAP)
    deg = round((J.rd(5, 56, 2) - r0) / TPD, 1)
    ev("twist", source="nudge.py keyboard", tq=850, deg=deg, roll_peak=round(pk, 1), elbow_peak=round(pel, 1), vmin=vmin)
    return deg, pk


def relax_roll(limit=4):
    for _ in range(24):
        l = J.load(5)
        if abs(l) <= limit: return
        p = J.rd(5, 56, 2); J.wr(5, 46, 2, 30); J.wr(5, 42, 2, int(p - 6 if l > 0 else p + 6)); time.sleep(0.3)


def main():
    cfg = load_cfg()
    import capguard
    os.environ["TC_TAKE"] = "pending"           # recording starts on the first MOTION key, stops after IDLE_STOP s quiet
    TK = {"take": None, "last": time.time()}
    def ensure_take():
        TK["last"] = time.time()
        if TK["take"] is None:
            sys.stdout.write("\rstarting camera take (~25 s) ...".ljust(200)); sys.stdout.flush()
            lj = os.path.join(str(tc_config.CAPTURE_DIR), "lock.json")   # re-tune exposure if > 30 min old
            if not os.path.exists(lj) or time.time() - os.path.getmtime(lj) > 1800:     # (10/30 night takes were too dark)
                sys.stdout.write("\rtuning dock-cam exposure (~60 s) ...".ljust(200)); sys.stdout.flush()
                import subprocess
                subprocess.run([sys.executable, os.path.join(HERE, "capture.py"), "check", "--shots"], capture_output=True, timeout=180)
            try: TK["lock"] = json.load(open(lj))
            except Exception: TK["lock"] = None
            TK["take"] = capguard.require("nudge.py") or "none"; os.environ["TC_TAKE"] = str(TK["take"])
            ev("take-start", take=TK["take"])
    def idle_stop():
        if TK["take"] not in (None, "none") and time.time() - TK["last"] > IDLE_STOP and C.idle():
            capguard.release("nudge.py"); ev("take-stop", take=TK["take"], why="idle"); TK["take"] = None
            return "idle: take stopped (next move restarts it)"
        return ""
    take = None
    trace = J.start_trace("nudge")
    for i in range(1, 6):                                   # torque on WITHOUT a jump: goal := present first
        if not J.rd(i, 40, 1):
            J.wr(i, 42, 2, J.rd(i, 56, 2)); J.wr(i, 40, 1, 1)
    pan_lim0 = J.rd(1, 48, 2)
    C = Ctl()
    step_mm, step_roll, speed = 1.0, 2, 3.0                 # mm/s
    soft = False
    tgt = pose(C.meas)                                      # commanded x y z az (not the sagging measurement)
    ev("start", take=take, trace=trace, pose=list(np.round(tgt, 3)))

    def go(t_new, mm_s=None, label=""):
        nonlocal tgt
        t_new = np.asarray(t_new, float)
        L = np.linalg.norm(t_new[:3] - tgt[:3]); n = max(1, int(np.ceil(L / 2.0)))
        seed = C.target_q.copy(); qs = []
        for s in np.linspace(0, 1, n + 1)[1:]:
            q, ok = ik4(tgt + (t_new - tgt) * s, seed)
            if not ok or not in_range(q):
                return f"no IK / out of range at {np.round(tgt + (t_new - tgt) * s, 1)}: not moving"
            qs.append(q); seed = q
        for q in qs:
            C.add(q, (L / n) / (mm_s or speed))
        C.target_q = qs[-1].copy(); tgt = t_new
        return ""

    def wait_idle(tmax=60):
        t0 = time.time()
        while not C.idle() and time.time() - t0 < tmax and not C.frozen:
            if msvcrt.kbhit():
                C.freeze("key abort"); return False
            time.sleep(0.05)
        return not C.frozen

    msg = "ready. O = standoff, keys in the header (python nudge.py -h)"
    MOVES = {"w": (1, +1), "s": (1, -1), "a": (0, -1), "d": (0, +1), "r": (2, +1), "f": (2, -1)}
    try:
        while True:
            m = pose(C.meas)
            line = (f"\r{'FROZEN ' + C.why + ' | ' if C.frozen else ''}face x{m[0]:6.1f} y{m[1]:6.1f} z{m[2]:6.1f} az{m[3]:+.3f}"
                    f" roll{J.rd(5, 56, 2):5d} | load L{C.ld[1]:+3.0f} E{C.ld[2]:+3.0f} F{C.ld[3]:+3.0f} R{C.ld[4]:+3.0f}"
                    f" | {C.temp}C | step {step_mm}mm/{step_roll}d {speed:.0f}mm/s{' SOFT' if soft else ''} | {msg[:60]}")
            sys.stdout.write(line.ljust(200)[:200]); sys.stdout.flush()
            if not msvcrt.kbhit():
                msg = idle_stop() or msg
                time.sleep(0.1); continue
            k = msvcrt.getwch(); kl = k.lower(); before = list(np.round(pose(C.meas), 2)); msg = k
            if kl in "wsadrftgqeoilzuyk":
                ensure_take()
            else:
                TK["last"] = time.time()
            if k == " ":
                if C.frozen: C.unfreeze(); tgt = pose(C.meas); msg = "unfrozen"
                else: C.freeze("space"); msg = "frozen"
            elif kl == "x":
                break
            elif C.frozen:
                msg = "FROZEN: space to resume"; continue
            elif kl in MOVES:
                ax, sg = MOVES[kl]; t2 = tgt.copy(); t2[ax] += sg * step_mm; msg = go(t2) or f"{kl} {sg*step_mm:+.1f}"
            elif kl in "tg":
                t2 = tgt.copy(); t2[3] += 0.01 if kl == "t" else -0.01; msg = go(t2) or f"az {t2[3]:+.3f}"
            elif kl in "qe":
                r = J.rd(5, 56, 2) + (step_roll if kl == "e" else -step_roll) * TPD
                J.wr(5, 41, 1, 10); J.wr(5, 46, 2, 80); J.wr(5, 42, 2, int(r)); msg = f"roll -> {int(r)}"
            elif k in STEPS:
                step_mm, step_roll = STEPS[k]
            elif k == "[":
                speed = max(0.5, speed / 1.5)
            elif k == "]":
                speed = min(10.0, speed * 1.5)
            elif kl == "k":
                soft = not soft; J.wr(1, 48, 2, 300 if soft else pan_lim0); msg = f"pan soft {soft}"
            elif kl == "o":
                cfg = load_cfg(); rdy, so = cfg["ready"], cfg["standoff"]
                msg = "roll to entry"; roll_goto(cfg["entry"])
                if m[1] < rdy[1] - 20 or abs(m[0] - rdy[0]) > 40:          # far away (rest): JOINT move to the ready pose
                    q, ok = ik4(rdy, [-86.8, 31.3, 57.3, -60.5, C.meas[4]])
                    if not ok or not in_range(q):
                        msg = "no IK for ready pose"; continue
                    if C.plan[1] < -60:                  # folded at rest: unfold UP first (straight to ready drags on the mat)
                        unf = np.array([C.plan[0], -50.0, 80.0, 0.0])
                        C.add(unf, np.abs(unf - C.plan).max() / 8.0)
                        src = unf
                    else:
                        src = C.plan
                    span = np.abs(q[:4] - src).max(); C.add(q, span / 8.0)      # 8 deg/s
                    C.target_q = q.copy(); tgt = np.array(rdy, float)
                    if not wait_idle(): msg = "aborted on the way to ready"; continue
                msg = go(so, 4.0) or "standoff"; wait_idle()
            elif kl == "i":
                base = None; pushed = 0.0; msg = "pushing in"
                while pushed < 30:
                    t2 = tgt.copy(); t2[1] += 0.5; e = go(t2, 1.5)
                    if e: msg = e; break
                    if not wait_idle(10): msg = "push aborted"; break
                    time.sleep(0.15); pushed += 0.5
                    el = abs(C.ld[2]); base = el if base is None else min(base, el)
                    lag = C.plan - C.meas[:4]
                    if el > base + 10 or (np.abs(lag[1:]) > 1.5).any():
                        msg = f"CONTACT after {pushed:.1f} mm (elbow {el:.0f}% vs {base:.0f}%, lag {np.round(lag,1)})"; break
            elif kl == "l":
                ang, pk = twist_to_stall(); msg = f"twist {ang} deg peak {pk:.0f}% -> " + ("LOCK?" if pk >= 30 and ang > -75 else "spun free")
                ev("twist", deg=ang, peak=pk)
            elif kl == "u":
                msg = f"roll at {roll_goto(load_cfg()['entry'])}"
            elif kl == "z":
                relax_roll(); msg = f"roll load {J.load(5):.0f}%"
            elif kl == "y":
                t2 = tgt.copy(); t2[2] += 20; msg = go(t2, 1.0) or "lifting 20 mm"
            elif kl == "p":
                cfg = load_cfg(); cfg["standoff"] = [round(float(v), 3) for v in tgt]
                json.dump(cfg, open(CFG, "w"), indent=1); msg = f"standoff saved {cfg['standoff']}"
            elif kl == "m":
                sys.stdout.write("\nmark: "); sys.stdout.flush(); note = input().strip(); msg = f"marked '{note}'"
                ev("mark", note=note, pose=before, roll=J.rd(5, 56, 2), loads=list(C.ld))
                with open(os.path.join(HERE, "logs", "takes_index.jsonl"), "a") as f:     # one whole line, append (shared w/ so101real)
                    f.write(json.dumps({"kind": "attempt", "take": TK["take"], "script": "nudge.py",
                                        "wall": time.strftime("%Y-%m-%d %H:%M:%S"), "t": round(time.time(), 1),
                                        "note": note, "wiggle": "wiggle" in note.lower(), "pose": before, "cam_lock": TK.get("lock")}) + "\n")
                try: capguard._mark("nudge", note)
                except Exception: pass
                continue
            ev("key", key=k, before=before, cmd=list(np.round(tgt, 3)), roll=J.rd(5, 56, 2), loads=list(C.ld), msg=msg)
    finally:
        C.run = False; C.th.join(1.0); J.hold()
        if soft: J.wr(1, 48, 2, pan_lim0)
        C.csv.close(); ev("quit", pose=list(np.round(pose(read5()), 2)))
        print("\nquit: torque ON, holding. log", C.csv.name)
        try: capguard.release("nudge.py")
        except Exception: pass


if __name__ == "__main__":
    if "-h" in sys.argv: print(__doc__); sys.exit(0)
    main()

# SPDX-License-Identifier: GPL-3.0-or-later
"""Browser controller for the dock pick (10/31, swap-desk). Same motion core as nudge.py (Ctl / ik4 / safety), driven
from a web page: live dock cam, big buttons, keyboard (WASD/RF/TG/QE).

    python nudge_web.py [--port 8765]     -> open http://localhost:8765

LINE UP (auto): roll to entry, unfold from rest, go to `lineup` = 10 mm in front of where the master last sat against a
docked tool (cfg "tool_seen"), pitch level. operator then adjusts, PUSH IN (to contact), LOCK (twist to stall), wiggle, LIFT.
Every command + pose -> logs/nudge_events.jsonl; marks -> logs/takes_index.jsonl (kind=attempt). Recording starts on the
first motion command and stops after IDLE_STOP s with no commands.
"""
import glob, json, os, queue, subprocess, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import numpy as np
import nudge as N
import jog as J

HERE = N.HERE
import tc_config  # noqa: E402  (machine-specific settings: config.json, see config.example.json)
CAPDIR = str(tc_config.CAPTURE_DIR)
PORT = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 8765
TOOL_SEEN = [52.8, 243.5, 72.4, 0.0]          # 10/30 20:30: master face parked against the docked tool (encoders)


class HiLog:
    """11/02 eve (so101-overlay): background logger of MEASURED servo positions (sync read, J.rd_pos5) at up to `hz`,
    PC clock (time.time, same clock as the sync_beep events). logs/hilog_<tag>_<stamp>.csv:
    t, pan..roll (deg), raw ticks 1-5, mark (beep rows)."""
    def __init__(self, tag, hz=100):
        import csv
        self.path = os.path.join(HERE, "logs", f"hilog_{os.path.basename(str(tag))}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        self.f = open(self.path, "w", newline="", buffering=1); self.w = csv.writer(self.f)
        self.w.writerow(["t", "pan", "lift", "elbow", "flex", "roll", "tick1", "tick2", "tick3", "tick4", "tick5", "mark"])
        self.dt = 1.0 / float(hz); self.run = True; self._mark = ""; self.n = 0
        self.th = threading.Thread(target=self.loop, daemon=True); self.th.start()
        N.ev("hilog", state="start", file=self.path, hz=hz)

    def mark(self, m):
        self._mark = m

    def loop(self):
        nxt = time.time()
        while self.run:
            try:
                tk = J.rd_pos5(); t = time.time(); d = N.d5_from_ticks(tk)
                m, self._mark = self._mark, ""
                self.w.writerow([f"{t:.4f}", *[f"{x:.3f}" for x in d], *tk, m]); self.n += 1
            except Exception:
                pass
            nxt += self.dt; time.sleep(max(0.0, nxt - time.time()))
            if time.time() - nxt > 0.5: nxt = time.time()

    def stop(self):
        self.run = False; self.th.join(1.0); self.f.close()
        N.ev("hilog", state="stop", file=self.path, rows=self.n)


class Desk:
    def __init__(self):
        import capguard; self.capguard = capguard
        for i in range(1, 6):
            if not J.rd(i, 40, 1):
                J.wr(i, 42, 2, J.rd(i, 56, 2)); J.wr(i, 40, 1, 1)
        self.pan_lim0 = J.rd(1, 48, 2)
        self.C = N.Ctl(); self.tgt = N.pose(self.C.meas)
        if not J.FAKE: J.wr(5, 48, 2, N.ROLL_CAP)           # 11/02: start capped (EEPROM default is 1000)
        self.step_mm, self.step_roll, self.speed, self.soft = 1.0, 2, 3.0, False
        self.take, self.lock, self.last = None, None, time.time()
        self.msg, self.busy, self.abort, self.intent = "ready", "", False, "practice"
        self.q = queue.Queue(); threading.Thread(target=self.worker, daemon=True).start()
        threading.Thread(target=self.idler, daemon=True).start()
        N.ev("web-start", pose=list(np.round(self.tgt, 3)))

    # ---- recording ----
    def ensure_take(self):
        self.last = time.time()
        if self.take is not None:
            return
        lj = os.path.join(CAPDIR, "lock.json")
        if not J.FAKE and (not os.path.exists(lj) or time.time() - os.path.getmtime(lj) > 1800):
            self.busy = "tuning camera exposure (~60 s)"
            subprocess.run([sys.executable, os.path.join(HERE, "capture.py"), "check", "--shots"], capture_output=True, timeout=180)
        try: self.lock = json.load(open(lj))
        except Exception: self.lock = None
        self.busy = "starting camera take (~25 s)"
        self.take = self.capguard.require("nudge_web.py") or "none"; os.environ["TC_TAKE"] = str(self.take)
        N.ev("take-start", take=self.take, cam_lock=self.lock)
        if self.intent != "practice": self.intent_row()          # new take inherits the session's intent

    def idler(self):
        while True:
            time.sleep(5)
            if self.take not in (None, "none") and not self.busy and self.C.idle() and time.time() - self.last > N.IDLE_STOP:
                self.capguard.release("nudge_web.py"); N.ev("take-stop", take=self.take, why="idle"); self.take = None
                self.msg = "idle 5 min: recording stopped (next move restarts it)"

    def attempt_row(self, note):
        with open(os.path.join(HERE, "logs", "takes_index.jsonl"), "a") as f:     # one whole line, append (shared w/ so101real)
            f.write(json.dumps({"kind": "attempt", "take": self.take, "script": "nudge_web.py", "intent": self.intent,
                                "wall": time.strftime("%Y-%m-%d %H:%M:%S"), "t": round(time.time(), 1), "note": note,
                                "wiggle": "wiggle" in note.lower(), "pose": list(np.round(N.pose(self.C.meas), 2)),
                                "cam_lock": self.lock}) + "\n")

    def intent_row(self):
        with open(os.path.join(HERE, "logs", "takes_index.jsonl"), "a") as f:     # count_attempts.py shape (pm-eng 10/31)
            f.write(json.dumps({"t": time.strftime("%Y-%m-%d %H:%M:%S"), "kind": "intent", "take": self.take,
                                "intent": self.intent}) + "\n")

    # ---- motion helpers ----
    def go(self, t_new, mm_s=None):
        """Straight line with ONE quintic time law (peak speed = mm_s), IK every ~0.1 s, linear between samples.
        11/01: was 2 mm pieces each with its own start/stop (stop-go every 2 mm, capped at 8 mm/s)."""
        C = self.C; t_new = np.asarray(t_new, float); v = mm_s or self.speed
        L = np.linalg.norm(t_new[:3] - self.tgt[:3]); da = abs(t_new[3] - self.tgt[3])
        T = max(0.3, 1.875 * L / v, da / 0.05); n = max(1, int(np.ceil(T / 0.1)))
        seed = C.target_q.copy(); qs = []
        for k in range(1, n + 1):
            tau = k / n; s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
            q, ok = N.ik4(self.tgt + (t_new - self.tgt) * s, seed)
            if not ok or not N.in_range(q):
                return f"can't reach {np.round(self.tgt + (t_new - self.tgt) * s, 1)}: not moving"
            qs.append(q); seed = q
        for q in qs:
            C.add(q, T / n, lin=True)
        C.target_q = qs[-1].copy(); self.tgt = t_new
        return ""

    def go_via(self, pts, vs, r=3.0, acc=80.0):
        """11/01e: ONE continuous move through waypoints (stops only at the end). Corners rounded by r mm
        (<= 40 % of each leg); per-leg peak speed vs[i] mm/s; accel limited to acc mm/s^2 (fwd/back pass)."""
        C = self.C; P = [self.tgt.copy()] + [np.asarray(p, float) for p in pts]
        dense, vlim = [P[0]], [vs[0]]
        def line(a, b, v):
            n = max(1, int(np.ceil(np.linalg.norm(b[:3] - a[:3]) / 0.5)))
            for k in range(1, n + 1): dense.append(a + (b - a) * k / n); vlim.append(v)
        cur = P[0]
        for i in range(1, len(P)):
            a, b = P[i - 1], P[i]
            if i < len(P) - 1:
                c = P[i + 1]; l1 = np.linalg.norm(b[:3] - a[:3]); l2 = np.linalg.norm(c[:3] - b[:3])
                rr = min(r, 0.4 * l1, 0.4 * l2)
                e1 = b - (b - a) * (rr / l1) if l1 > 1e-6 else b.copy()
                e2 = b + (c - b) * (rr / l2) if l2 > 1e-6 else b.copy()
                line(cur, e1, vs[i - 1])
                vc = min(vs[i - 1], vs[i], np.sqrt(acc * max(rr, 0.2)))      # v^2/R <= acc on the fillet
                for k in range(1, 9):
                    t = k / 8; dense.append((1 - t)**2 * e1 + 2 * (1 - t) * t * b + t**2 * e2); vlim.append(vc)
                cur = e2
            else:
                line(cur, b, vs[i - 1])
        D = np.array(dense); ds = np.r_[0, np.linalg.norm(np.diff(D[:, :3], axis=0), axis=1)]
        v = np.array(vlim, float); v[0] = 0; v[-1] = 0
        for k in range(1, len(v)): v[k] = min(v[k], np.sqrt(v[k - 1]**2 + 2 * acc * ds[k]))
        for k in range(len(v) - 2, -1, -1): v[k] = min(v[k], np.sqrt(v[k + 1]**2 + 2 * acc * ds[k + 1]))
        vm = np.maximum((v[1:] + v[:-1]) / 2, 0.3); t = np.r_[0, np.cumsum(ds[1:] / vm)]
        T = t[-1]; n = max(1, int(np.ceil(T / 0.1))); seed = C.target_q.copy(); qs = []
        for k in range(1, n + 1):
            tk = T * k / n; j = min(np.searchsorted(t, tk), len(t) - 1); j0 = max(j - 1, 0)
            f = 0 if t[j] == t[j0] else (tk - t[j0]) / (t[j] - t[j0]); x = D[j0] + (D[j] - D[j0]) * f
            q, ok = N.ik4(x, seed)
            if not ok or not N.in_range(q): return f"can't reach {np.round(x, 1)}: not moving"
            qs.append(q); seed = q
        for q in qs: C.add(q, T / n, lin=True)
        C.target_q = qs[-1].copy(); self.tgt = P[-1].copy()
        return ""

    def wait_idle(self, tmax=90, grace=3.0):
        t0 = time.time()
        while not self.C.idle() and time.time() - t0 < tmax and not self.C.frozen:
            if self.abort:
                self.C.freeze("stopped"); return False
            time.sleep(0.05)
        if self.C.frozen: return False
        t1 = time.time()
        while time.time() - t1 < grace and (np.abs(self.C.plan - self.C.meas[:4]) > 1.0).any():
            if self.abort or self.C.frozen: return False
            time.sleep(0.05)
        return not self.C.frozen

    def tool_on(self):
        return bool(N.load_cfg().get("tool_on", False))

    def set_tool(self, on):
        cfg = N.load_cfg(); cfg["tool_on"] = bool(on); json.dump(cfg, open(N.CFG, "w"), indent=1)

    def joint_to(self, q):
        C = self.C
        if C.plan[1] < -60:                              # folded at rest: unfold UP first
            unf = np.array([C.plan[0], -50.0, 80.0, 0.0]); C.add(unf, np.abs(unf - C.plan).max() / 8.0); src = unf
        else:
            src = C.plan
        C.add(q, np.abs(q[:4] - src).max() / 8.0); C.target_q = q.copy()

    def twist(self):
        why = self.twist_refusal()
        if why:                                          # 11/02 pm-eng: 850 only when the elbow is unloaded and no hand move runs
            print(f"WARNING twist refused: {why}", flush=True); N.ev("twist-refused", why=why, loads=list(np.round(self.C.ld, 1)))
            return None, why
        r0 = J.rd(5, 56, 2)
        J.wr(5, 48, 2, 850); J.wr(5, 41, 1, 10); J.wr(5, 46, 2, 60); J.wr(5, 42, 2, int(r0 - 80 * N.TPD))
        t0, last, still, pk = time.time(), r0, 0, 0
        pel, vmin = abs(J.load(3)), J.rd(2, 62, 1) / 10
        while time.time() - t0 < 20 and not self.abort:
            time.sleep(0.1); p = J.rd(5, 56, 2); pk = max(pk, abs(J.load(5)))
            pel = max(pel, abs(J.load(3))); vmin = min(vmin, J.rd(2, 62, 1) / 10)
            still = still + 1 if abs(p - last) < 2 else 0; last = p
            if still >= 6: break
        J.wr(5, 42, 2, J.rd(5, 56, 2)); J.wr(5, 48, 2, N.ROLL_CAP)
        self.last_twist = {"elbow_peak": round(pel, 1), "vmin": vmin}
        return round((J.rd(5, 56, 2) - r0) / N.TPD, 1), pk

    def twist_refusal(self):
        el = abs(float(self.C.ld[2]))
        if el > 45: return f"elbow load {el:.0f} % > 45 % (11/01 13:21 cut-out followed a 48 % elbow press + roll 55 %)"
        if os.path.exists(os.path.join(HERE, "contact_active.lock")): return "a contact/hand move is active (contact_active.lock)"
        return ""

    # ---- commands ----
    MOVES = {"in": (1, 1), "out": (1, -1), "left": (0, -1), "right": (0, 1), "up": (2, 1), "down": (2, -1)}
    def run_path(self, name):
        """Steps (paths/<name>.json, list):
          {"to": [x,y,z,az], "v": mm/s}                 straight line, waits for measured arrival (3 s grace on contact)
          {"via": [[x,y,z,az,v],...], "r": 3, "acc": 80}  one blended move, stops only at the end (11/01e)
          {"joint": [x,y,z,az], "dps": deg/s}           IK + joint-space swing (free space only)
          {"roll": ticks, "spd": 300, "tq": 1000, "secs": 4}             roll to an absolute tick
          {"roll_stall": +-ticks, "spd": 150, "tq": 1000, "secs": 4}     roll that way until it stops moving; remembers stall
          {"roll_from_stall": +-ticks, "spd": 300}                       absolute = last stall + ticks
          {"dwell": s}
        Every step is logged as a 'path-step' event (pose, roll, loads)."""
        name, _, sc = name.partition("@"); scale = float(sc or 1.0)          # "pick_v2@1.5" = all speeds x1.5
        C = self.C; p = os.path.join(HERE, "paths", os.path.basename(name) + ".json")
        steps = json.load(open(p)); stall = None; C.fast_log = True
        try:
            return self._run_path(name, steps, scale)
        finally:
            C.fast_log = False
            if getattr(self, "hilog", None): self.hilog.stop(); self.hilog = None      # never leave the logger running

    def halt(self):
        """Stop the planned motion where the plan is now (no freeze, no jump)."""
        C = self.C
        with C.lock:
            C.segs.clear(); C.seg = None
        C.target_q[:4] = C.plan.copy(); self.tgt = np.array(N.pose(np.r_[C.plan, C.meas[4]]), float)

    def _run_path(self, name, steps, scale):
        C = self.C; stall = None
        for i, s in enumerate(steps):
            if self.abort or C.frozen: return f"path {name} stopped at step {i}"
            self.busy = f"{name} {i + 1}/{len(steps)}"
            if s.get("beep"):
                subprocess.Popen(["powershell", "-NoProfile", "-Command", f"[console]::beep({s['beep']},250)"])
            if "to" in s and "until" in s:                    # creep until a joint load rises by 'until' over its start value
                base = C.ld.copy(); e = self.go(s["to"], s.get("v", 4.0) * scale)
                if e: return f"path {name} step {i}: {e}"
                hit = None
                while not C.idle() and not self.abort and not C.frozen:
                    d = C.ld - base
                    for k, lim in s["until"].items():
                        j = ["pan", "lift", "elbow", "flex", "roll"].index(k)
                        if abs(d[j]) >= lim: hit = (k, round(float(C.ld[j])))
                    if hit: self.halt(); break
                    time.sleep(0.02)
                N.ev("contact", name=name, i=i, hit=hit, pose=list(np.round(N.pose(C.meas), 2)))
                if not self.wait_idle(grace=0.3): return f"path {name} stopped in step {i}"
            elif "via" in s:                                  # [[x,y,z,az,v], ...] one blended move
                e = self.go_via([w[:4] for w in s["via"]], [w[4] * scale for w in s["via"]], s.get("r", 3.0), s.get("acc", 80.0) * scale)
                if e: return f"path {name} step {i}: {e}"
                if not self.wait_idle(grace=s.get("grace", 1.5)): return f"path {name} stopped in step {i}"
            elif "to" in s:
                e = self.go(s["to"], s.get("v", 5.0) * scale)
                if e: return f"path {name} step {i}: {e}"
                if not self.wait_idle(grace=s.get("grace", 1.5)): return f"path {name} stopped in step {i}"
            elif "joint" in s:
                q, ok = N.ik4(s["joint"], C.target_q.copy())
                if not ok or not N.in_range(q): return f"path {name} step {i}: can't reach {s['joint']}"
                src = C.plan; C.add(q, max(0.3, np.abs(q[:4] - src).max() / s.get("dps", 20.0))); C.target_q = q.copy()
                self.tgt = np.array(s["joint"], float)
                if not self.wait_idle(): return f"path {name} stopped in step {i}"
            elif "roll" in s or "roll_from_stall" in s:
                goal = s["roll"] if "roll" in s else (stall if stall is not None else J.rd(5, 56, 2)) + s["roll_from_stall"]
                J.wr(5, 48, 2, min(s.get("tq", N.ROLL_CAP), 600)); J.wr(5, 41, 1, 20); J.wr(5, 46, 2, s.get("spd", 300)); J.wr(5, 42, 2, int(goal))
                t0 = time.time()
                while time.time() - t0 < s.get("secs", 4) and abs(J.rd(5, 56, 2) - goal) > 10 and not self.abort:
                    time.sleep(0.05)
                J.wr(5, 48, 2, N.ROLL_CAP)
            elif "roll_stall" in s:
                r0 = J.rd(5, 56, 2)
                J.wr(5, 48, 2, min(s.get("tq", N.ROLL_CAP), 600)); J.wr(5, 41, 1, 20); J.wr(5, 46, 2, s.get("spd", 150))
                J.wr(5, 42, 2, int(r0 + s["roll_stall"]))
                t0, last, still = time.time(), r0, 0
                while time.time() - t0 < s.get("secs", 4) and not self.abort:
                    time.sleep(0.05); r = J.rd(5, 56, 2)
                    still = still + 1 if abs(r - last) < 2 else 0; last = r
                    if still >= 8 and time.time() - t0 > 0.6: break
                stall = J.rd(5, 56, 2)
                if s.get("hold", False): pass                      # keep pushing toward the goal (more torque)
                else: J.wr(5, 42, 2, stall)
                J.wr(5, 48, 2, N.ROLL_CAP)
            elif "twist_lock" in s:                             # the operator's fix for the ramp stall: back off, nudge, re-twist
                tl = s["twist_lock"]; tries = []
                nudges = tl.get("nudges", [[0, 0, 0, 0.02], [0, 2, 0, 0], [0, 2, 0, 0.02], [0, 0, -2, 0]])
                for k in range(tl.get("tries", 4)):
                    r0 = J.rd(5, 56, 2)
                    J.wr(5, 48, 2, tl.get("tq", 550)); J.wr(5, 41, 1, tl.get("acc", 20)); J.wr(5, 46, 2, tl.get("spd", 150)); J.wr(5, 42, 2, int(tl["target"] - 300))   # 11/01 brownout at 1000
                    t0, last, still, pk = time.time(), r0, 0, 0; rtrace = []
                    while time.time() - t0 < tl.get("secs", 5) and not self.abort:
                        time.sleep(0.05); r = J.rd(5, 56, 2); lr = J.load(5); pk = max(pk, abs(lr)); rtrace.append([round(time.time() - t0, 2), r, round(lr)])
                        still = still + 1 if abs(r - last) < 2 else 0; last = r
                        if still >= 8 and time.time() - t0 > 0.6: break
                    stall = J.rd(5, 56, 2); J.wr(5, 42, 2, stall); J.wr(5, 48, 2, N.ROLL_CAP); tries.append([stall, round(pk)])
                    N.ev("twist-trace", name=name, i=i, k=k, trace=rtrace)
                    if stall <= tl.get("ok_below", 2750) or self.abort: break
                    roll_goto_ok = N.roll_goto(stall + tl.get("backoff", 80), speed=300, secs=2)
                    nd = nudges[k % len(nudges)]
                    e = self.go(self.tgt + np.array(nd, float), 4.0)
                    if e or not self.wait_idle(grace=0.3): return f"path {name} twist nudge failed: {e}"
                N.ev("twist-lock", name=name, tries=tries)
                if stall > tl.get("ok_below", 2750):
                    return f"path {name}: NO LOCK after {len(tries)} twists {tries} (stopped before lifting)"
            elif "qtraj" in s:                                  # so101_personality: smooth validated primitive (FROZEN vendored copy, 11/02)
                sys.path.insert(0, os.path.join(HERE, "qtraj_vendor"))
                import qtraj_step; e = qtraj_step.play(self, s["qtraj"], scale * s.get("scale", 1.0))
                if e: return f"path {name} step {i}: {e}"
            elif "contact" in s:                                # hand_unlock / hand_lock: the operator's go token required
                sys.path.insert(0, os.path.join(HERE, "qtraj_vendor"))
                import qtraj_step; e = qtraj_step.CONTACT[s["contact"]](self)
                if e: return f"path {name} step {i}: {e}"
            elif "hilog" in s:                                  # 11/02 eve overlay: MEASURED positions at up to 100 Hz
                if s["hilog"] == "start":
                    if getattr(self, "hilog", None): self.hilog.stop()
                    self.hilog = HiLog(s.get("tag", name), s.get("hz", 100))
                elif getattr(self, "hilog", None):
                    self.hilog.stop(); self.hilog = None
            elif "sync_beep" in s:                              # beep NOW (winsound, no PowerShell spawn lag) + exact timestamp
                import winsound
                t = time.time(); threading.Thread(target=winsound.Beep, args=(int(s["sync_beep"]), int(s.get("ms", 300))), daemon=True).start()
                N.ev("sync_beep", t_beep=round(t, 4), f=s["sync_beep"], path=name)
                if getattr(self, "hilog", None): self.hilog.mark(f"beep {s['sync_beep']} t={t:.4f}")
            elif "dwell" in s:
                time.sleep(s["dwell"])
            elif "expect" in s:                                 # 11/02: stop before twisting if the push stopped short (tool sits off its seat)
                m = N.pose(C.meas); ex = s["expect"]
                bad = [f"{k[0]} {m['xyz'.index(k[0])]:.1f} < {v}" for k, v in ex.items() if k.endswith("_min") and m['xyz'.index(k[0])] < v]
                if "along" in ex:                               # dock-frame depth (dockframe.py): (face - o) . u >= min
                    al = ex["along"]; d = (m[0] - al["o"][0]) * al["u"][0] + (m[1] - al["o"][1]) * al["u"][1]
                    if d < al["min"]: bad.append(f"depth {d:.1f} < {al['min']}")
                if bad:
                    N.ev("expect-fail", name=name, i=i, pose=list(np.round(m, 2)), why=bad)
                    if "back" in s:                             # ease off: don't sit pushing at elbow 60 %
                        self.go(s["back"], 5.0); self.wait_idle(grace=0.5)
                    return f"path {name} step {i}: SHORT {', '.join(bad)} (tool off its seat?)"
            elif "weigh" in s:                                  # mean load over s["weigh"] seconds = tool on/off label
                time.sleep(0.4); acc = []
                t0 = time.time()
                while time.time() - t0 < s["weigh"]: acc.append(C.ld.copy()); time.sleep(0.03)
                wt = list(np.round(np.mean(acc, 0), 1)); self.last_weigh = {"tag": s.get("tag", ""), "load": wt}
                N.ev("weigh", name=name, tag=s.get("tag", ""), load=wt, pose=list(np.round(N.pose(C.meas), 2)))
            N.ev("path-step", name=name, i=i, step=s, pose=list(np.round(N.pose(C.meas), 2)), roll=J.rd(5, 56, 2),
                 loads=list(C.ld), stall=stall)
        return f"path {name} done ({len(steps)} steps), last stall {stall}"

    MOTION = set(MOVES) | {"path", "tipup", "tipdown", "rollm", "rollp", "lineup", "autopick", "overdock", "push", "lock", "unlock", "relax", "lift", "soft"}

    def submit(self, cmd, arg=""):
        if cmd == "stop":
            self.abort = True; self.C.freeze("STOP button"); self.msg = "STOPPED (press Resume)"; return
        if cmd == "resume":
            self.C.unfreeze(); self.tgt = N.pose(self.C.meas); self.abort = False; self.msg = "resumed"; return
        if cmd == "step":
            self.step_mm, self.step_roll = N.STEPS.get(arg, (1.0, 2)); return
        if cmd == "speed":
            self.speed = float(np.clip(float(arg), 0.5, 10)); return
        if cmd in self.MOVES and self.q.qsize() > 3:
            return                                       # don't pile up held-key repeats
        self.q.put((cmd, arg))

    def worker(self):
        while True:
            cmd, arg = self.q.get()
            if self.C.frozen and cmd != "mark":
                self.msg = "FROZEN: press Resume first"; continue
            before = list(np.round(N.pose(self.C.meas), 2))
            try:
                if cmd in self.MOTION:
                    self.ensure_take()
                self.last = time.time(); self.busy = cmd
                self.msg = self.run(cmd, arg) or self.msg
            except Exception as ex:
                self.C.freeze(f"error {ex}"); self.msg = f"error: {ex}"
            finally:
                self.busy = ""
            N.ev("web", cmd=cmd, arg=arg, before=before, cmd_pose=list(np.round(self.tgt, 3)), roll=J.rd(5, 56, 2),
                 loads=list(self.C.ld), msg=self.msg)

    def run(self, cmd, arg):
        C = self.C
        if cmd in self.MOVES:
            ax, sg = self.MOVES[cmd]; t2 = self.tgt.copy(); t2[ax] += sg * self.step_mm
            return self.go(t2) or f"{cmd} {self.step_mm} mm"
        if cmd in ("tipup", "tipdown"):
            t2 = self.tgt.copy(); t2[3] += 0.01 if cmd == "tipup" else -0.01; return self.go(t2) or f"tip az {t2[3]:+.3f}"
        if cmd in ("rollm", "rollp"):
            r = J.rd(5, 56, 2) + (self.step_roll if cmd == "rollp" else -self.step_roll) * N.TPD
            J.wr(5, 41, 1, 10); J.wr(5, 46, 2, 80); J.wr(5, 42, 2, int(r)); return f"roll -> {int(r)}"
        if cmd == "soft":
            self.soft = not self.soft; J.wr(1, 48, 2, 300 if self.soft else self.pan_lim0); return f"pan soft {self.soft}"
        if cmd == "lineup":
            cfg = N.load_cfg(); seen = cfg.get("tool_seen", TOOL_SEEN)
            line = [seen[0], seen[1] - 10.0, seen[2], seen[3]]
            self.busy = "roll to entry"; N.roll_goto(cfg["entry"])
            m = N.pose(C.meas)
            if np.linalg.norm(m[:3] - np.array(line[:3])) > 40:          # far (rest): joint move to a ready pose behind it
                ready = [line[0], line[1] - 30.0, line[2] + 8.0, line[3]]
                q, ok = N.ik4(ready, [-86.8, 31.3, 57.3, -60.5, C.meas[4]])
                if not ok or not N.in_range(q):
                    return f"can't reach ready pose {ready}"
                self.busy = "unfold + swing to the dock"; self.joint_to(q); self.tgt = np.array(ready, float)
                if not self.wait_idle(): return "stopped on the way"
            self.busy = "straight in to the lineup point"
            e = self.go(line, 3.0)
            if e: return e
            if not self.wait_idle(): return "stopped"
            return f"LINED UP 10 mm in front of the last tool position {seen[:3]}. Adjust, then PUSH IN."
        if cmd == "path":                           # 11/01: keyframe runner, paths/<arg>.json (fast pick / reverse)
            return self.run_path(arg)
        if cmd == "autopick":                       # the operator's 10/31 17:55 lock, scripted (cfg chase_lock_1031)
            if self.tool_on():
                return "AUTO PICK refused: a tool is ON the master (use OVER DOCK + lower + UNLOCK, or click 'Tool is OFF' if wrong)"
            self.intent = "pick"; self.intent_row(); self.attempt_row("autopick start")
            if C.temp >= 52:
                return f"motors {C.temp} C: let them cool below 50 first"
            if np.linalg.norm(N.pose(C.meas)[:3] - np.array(N.load_cfg().get("tool_seen", TOOL_SEEN)[:3])) > 60:
                self.busy = "BEFORE shot (2 s, arm clear)"; time.sleep(2.0); self.attempt_row("before_shot")
            r = self.run("lineup", "")
            if not r.startswith("LINED UP"): return f"AUTO PICK stopped at LINE UP: {r}"
            self.busy = "pre-twist to entry 3293"; N.roll_goto(N.load_cfg()["entry"])
            t2 = self.tgt.copy(); t2[3] = 0.04; e = self.go(t2, 2.0)
            if e or not self.wait_idle(): return f"AUTO PICK stopped at tip-up: {e or C.why}"
            ystop = float(N.load_cfg().get("chase_lock_1031", {}).get("push_to_y", 257))
            self.busy = f"push in to the stop (y {ystop:.0f})"
            while self.tgt[1] < ystop:
                t2 = self.tgt.copy(); t2[1] = min(ystop, t2[1] + 1.0); e = self.go(t2, 1.5)
                if e or not self.wait_idle(10): return f"AUTO PICK stopped while pushing at y {N.pose(C.meas)[1]:.1f}: {e or C.why}"
                time.sleep(0.1)
                if abs(C.ld[2]) > 55 or (np.abs(C.plan - C.meas[:4])[1:] > 2.5).any():
                    break                                # at the stop (or caught): don't fight it
            at = N.pose(C.meas)
            self.busy = "twist to catch"; ang, pk = self.twist()
            if ang is None: self.busy = ""; return f"TWIST refused: {pk}"
            N.ev("twist", source="nudge_web auto", tq=850, deg=ang, roll_peak=pk, take=self.take, auto=True, **getattr(self, "last_twist", {}))
            if not (pk >= 30 and ang > -75):
                self.attempt_row(f"autopick miss: twist {ang} deg peak {pk:.0f}% at y {at[1]:.1f} z {at[2]:.1f}")
                return f"AUTO PICK: no catch (twist {ang} deg, peak {pk:.0f}%) at y {at[1]:.1f} z {at[2]:.1f}. Adjust and press LOCK, or UNLOCK + retry"
            self.busy = "relax twist"; N.relax_roll()
            self.busy = "ease out + up"; t2 = self.tgt.copy(); t2[1] -= 8; t2[2] += 10; t2[3] = 0.09; e = self.go(t2, 1.0)
            if e or not self.wait_idle(): return f"AUTO PICK: caught (twist {ang} deg, {pk:.0f}%) but ease-out stopped: {e or C.why}"
            self.busy = "lift 20"; t2 = self.tgt.copy(); t2[2] += 20; e = self.go(t2, 1.0)
            if e or not self.wait_idle(): return f"AUTO PICK: lift stopped: {e or C.why}"
            self.set_tool(True)
            self.attempt_row(f"autopick lifted: twist {ang} deg peak {pk:.0f}%, lift load {C.ld[1]:.0f}%")
            self.busy = "back away for the AFTER shot"; t2 = self.tgt.copy(); t2[1] -= 30; t2[2] -= 15; t2[3] = 0.05; e = self.go(t2, 3.0)
            if not e and self.wait_idle():
                self.busy = "AFTER shot (2 s, empty slot)"; time.sleep(2.0); self.attempt_row("after_shot")
            return f"AUTO PICK done: twist {ang} deg ({pk:.0f}%), lifted. Lift load {C.ld[1]:.0f}%. WIGGLE it on camera, then 'wiggle'."
        if cmd == "intent":
            self.intent = arg or "practice"; self.intent_row(); return f"intent: {self.intent}"
        if cmd == "push":
            base = None; pushed = 0.0
            while pushed < 25:
                t2 = self.tgt.copy(); t2[1] += 0.5; e = self.go(t2, 1.5)
                if e: return e
                if not self.wait_idle(10): return "push stopped"
                time.sleep(0.15); pushed += 0.5
                el = abs(C.ld[2]); base = el if base is None else min(base, el)
                lag = C.plan - C.meas[:4]
                if el > base + 10 or (np.abs(lag[1:]) > 1.5).any():
                    return f"CONTACT after {pushed:.1f} mm (elbow load {el:.0f}% vs {base:.0f}%)"
            return "pushed 25 mm, no contact"
        if cmd == "lock":
            ang, pk = self.twist()
            if ang is None: return f"TWIST refused: {pk}"
            N.ev("twist", source="nudge_web button", tq=850, deg=ang, roll_peak=pk, take=self.take, **getattr(self, "last_twist", {}))
            if pk >= 30 and ang > -75: self.set_tool(True)
            return f"twist {ang} deg, peak {pk:.0f}% -> " + ("LOCKED? wiggle it on camera, then mark 'wiggle'" if pk >= 30 and ang > -75 else "spun free (no lock)")
        if cmd == "tool_off":
            self.set_tool(False); return "marked: no tool on the master"
        if cmd == "tool_on":
            self.set_tool(True); return "marked: tool ON the master"
        if cmd == "overdock":                    # always UP then OVER, never in from the front (10/31 18:23 post hit)
            Z = 172.0; seen = N.load_cfg().get("tool_seen", TOOL_SEEN); YO = seen[1] + 4.6   # 18:27 hover that cleared
            m = N.pose(C.meas)
            if abs(m[0] - seen[0]) > 30 or m[1] < 180:
                return "OVER DOCK only works from in front of the dock (x ~53, y > 180). Steer it there first."
            if m[1] > 215 and m[2] < Z - 5:
                self.busy = "out to y 208 first"; t2 = self.tgt.copy(); t2[1] = 208.0; e = self.go(t2, 3.0)
                if e or not self.wait_idle(): return f"OVER DOCK stopped backing out: {e or C.why}"
            self.busy = "level"; t2 = self.tgt.copy(); t2[3] = 0.0; e = self.go(t2, 2.0)
            if e or not self.wait_idle(): return f"OVER DOCK stopped leveling: {e or C.why}"
            if self.tgt[2] < Z:
                self.busy = f"up to z {Z:.0f}"; t2 = self.tgt.copy(); t2[2] = Z; e = self.go(t2, 3.0)
                if e or not self.wait_idle(): return f"OVER DOCK stopped going up: {e or C.why}"
            self.busy = "over the dock"; t2 = self.tgt.copy(); t2[0] = seen[0]; t2[1] = YO; e = self.go(t2, 3.0)
            if e or not self.wait_idle(): return f"OVER DOCK stopped going over: {e or C.why}"
            return f"OVER DOCK: hovering at y {YO:.0f} z {Z:.0f}. Lower with F (1 mm), then UNLOCK and back out with S."
        if cmd == "unlock":
            self.set_tool(False)
            return f"roll back to {N.roll_goto(N.load_cfg()['entry'])}"
        if cmd == "relax":
            N.relax_roll(); return f"roll load {J.load(5):.0f}%"
        if cmd == "lift":
            t2 = self.tgt.copy(); t2[2] += 20; return self.go(t2, 1.0) or "lifting 20 mm"
        if cmd == "save_lineup":
            cfg = N.load_cfg(); cfg["tool_seen"] = [round(float(v), 2) for v in N.pose(C.meas)]
            json.dump(cfg, open(N.CFG, "w"), indent=1); return f"saved this as the tool position {cfg['tool_seen']}"
        if cmd == "mark":
            self.attempt_row(arg)
            try: self.capguard._mark("nudge", arg)
            except Exception: pass
            return f"marked '{arg}'"
        return f"unknown {cmd}"

    def state(self):
        C = self.C; m = N.pose(C.meas)
        return {"x": round(m[0], 1), "y": round(m[1], 1), "z": round(m[2], 1), "az": round(m[3], 3), "roll": J.rd(5, 56, 2), "kit": N.R.MASTER_EXT_MM, "volt": round(min(J.rd(i, 62, 1) for i in (2, 3)) / 10, 1),
                "load": {n: round(float(v)) for n, v in zip(["pan", "lift", "elbow", "flex", "roll"], C.ld)}, "temp": C.temp, "temps": getattr(C, "temps", None), "temps_raw": getattr(C, "temps_raw", None),
                "frozen": C.frozen, "why": C.why, "busy": self.busy, "msg": self.msg, "step": self.step_mm,
                "speed": round(self.speed, 1), "intent": self.intent, "tool_on": self.tool_on(), "soft": self.soft, "take": self.take,
                "weigh": getattr(self, "last_weigh", None)}


def latest_jpg():
    try:
        st = json.load(open(os.path.join(CAPDIR, "capture_state.json")))
        f = os.path.join(CAPDIR, f"tc_scene_1028_{st['stamp']}.mkv.latest.jpg")
        if os.path.exists(f) and time.time() - os.path.getmtime(f) < 5:
            return f
    except Exception:
        pass
    f = os.path.join(CAPDIR, "framing_scene.jpg")
    return f if os.path.exists(f) else None


D = None


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def send(self, code, body, ctype):
        self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query)
        if u.path == "/":
            return self.send(200, open(os.path.join(HERE, "nudge_web.html"), "rb").read(), "text/html; charset=utf-8")
        if u.path == "/state":
            return self.send(200, json.dumps(D.state()).encode(), "application/json")
        if u.path == "/frame":
            f = latest_jpg()
            if not f: return self.send(404, b"", "text/plain")
            for _ in range(5):
                try: return self.send(200, open(f, "rb").read(), "image/jpeg")
                except OSError: time.sleep(0.05)
            return self.send(503, b"", "text/plain")
        if u.path == "/cmd":
            D.submit(q.get("c", [""])[0], q.get("a", [""])[0])
            return self.send(200, b"ok", "text/plain")
        self.send(404, b"", "text/plain")


if __name__ == "__main__":
    D = Desk()
    print(f"nudge_web on http://localhost:{PORT}  master kit {N.R.MASTER_EXT_MM:+.0f} mm (master_config.json)")
    try:
        ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
    finally:
        D.C.run = False; J.hold()
        if D.soft: J.wr(1, 48, 2, D.pan_lim0)
        if D.take not in (None, "none"): D.capguard.release("nudge_web.py")

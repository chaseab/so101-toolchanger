# SPDX-License-Identifier: GPL-3.0-or-later
"""Back-to-back pick + put-back cycles with self-labels; stops at the first miss, abort or freeze.
    python cycle.py <pick_path> <unpick_path> <n>        (operator at the bench; nudge_web running)
Summary rows -> logs/cycle_runs.jsonl."""
import json, os, sys, time
import run_path as R

HERE = os.path.dirname(os.path.abspath(__file__))


def ok(log, want):
    return log and not log.get("abort") and "done" in str(log.get("msg")) and log.get("dock", [None])[0] == want


def fmode(log):
    """failure_mode text; a bad camera frame is camera_invalid (scored abort), never a pick/put-back failure"""
    d = (log or {}).get("dock") or [None]
    if d[0] == "camera_invalid": return ("camera_invalid: " + str(d[3] if len(d) > 3 else ""))[:120]
    return str((log or {}).get("abort") or (log or {}).get("msg") or d)[:120]


def snap(kind, tag):
    import urllib.request, camcheck
    a = urllib.request.urlopen(R.URL + "/frame", timeout=5).read(); time.sleep(0.6); b = urllib.request.urlopen(R.URL + "/frame", timeout=5).read()
    bad = camcheck.frame_problem(b, a)                  # 11/02 eve: never save a placeholder/frozen frame as a reference
    if bad: raise SystemExit(f"camera_invalid: {bad} (no refs saved, no motion)")
    open(os.path.join(HERE, "refs", f"dock_{kind}_{tag}.jpg"), "wb").write(b)


WIG_MIN = 0.97
SCORE = os.path.join(HERE, "..", "SCOREBOARD.csv")
RUN_ID = time.strftime("c%m%d_%H%M%S")
PROFILES = {"base": {"settle": "wig_settle", "wiggle": "wiggle_1102", "box": (0, 30, 0, 90)},                     # 10/10 on 11/02 (pick_v12 + unpick_v4)
            "cool_v1": {"settle": "wig_settle_cool1", "wiggle": "wiggle_cool1", "box": (0, 45, 0, 100)},
            "fast": {"settle": "wig_settle_fast", "wiggle": "wiggle_fast", "box": (0, 30, 0, 90)}}       # 11/02 eve: same shake, one blended via at 60 mm/s   # box: tool 30 mm lower; check saved frames         # pick_cool1 + unpick_cool1: seat+1 mm, 2 tries, z 160
PROF_NAME = sys.argv[sys.argv.index("--profile") + 1] if "--profile" in sys.argv else "base"
PROF = PROFILES[PROF_NAME]
COOL_TO = float(sys.argv[sys.argv.index("--cool-to") + 1]) if "--cool-to" in sys.argv else 45.0   # 11/02 pm-eng: 45 (was 42)
_streak = {"n": 0, "row": 0}


def nm(n, k):
    """pm-eng 11/02: --first-slow runs cycle 0 at half speed (new profile's first pass, operator watching)."""
    return f"{n}@0.5" if (k == 0 and "--first-slow" in sys.argv) else n


def arg(k, d):
    return sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d


# 11/02 pm-eng (TC_POLICY_PLAN_1102): trailing SCOREBOARD columns, blank unless passed on the command line
POLICY_COLS = ("--light", "--cam-mark", "--clutter", "--seed", "--split", "--cell", "--ckpt")


def score(action, log, ok_, wiggle="na", mode="", k=0, notes="", retry=False):
    """11/02 pm-eng: one SCOREBOARD.csv row per attempt (pick or put-back). in_a_row counts FIRST tries only:
    a retry row never extends the streak (the short push before it already reset it)."""
    import csv
    _streak["row"] += 1; _streak["n"] = (_streak["n"] + 1 if ok_ else 0) if not retry else 0
    log = log or {}
    clip = f"take {log.get('take')} @ {log.get('t')} +{log.get('secs')}s (dock cam)" if log.get("take") else ""
    with open(SCORE, "a", newline="") as f:
        csv.writer(f).writerow([time.strftime("%Y-%m-%d %H:%M:%S"), RUN_ID, arg("--kind", "streak"), "scripted", arg("--tool", "GT1_v4"),
                                action, _streak["row"], _streak["n"], "success" if ok_ else ("abort" if str(mode).startswith("camera_invalid") else "fail"), wiggle, mode,
                                arg("--dockpos", "main"), arg("--offset", "0"), log.get("temp"), log.get("vmin"), clip, "",
                                f"logs/cycle_runs.jsonl {RUN_ID} k{k}; logs/path_runs.jsonl {log.get('t')}", f"profile={PROF_NAME}" + (f"; {notes}" if notes else ""),
                                *[arg(k, "") for k in POLICY_COLS]])


TOOL_BOX = (0, 30, 0, 90)           # rows, cols of the 160x90 frame where the lifted tool sits (dock cam, 11/02 framing)
CROP_MIN = 0.85                     # 11/02: stable tool reads 0.93-0.95 with the arm 2 mm off (friction hysteresis); frames saved to refs/wiggle/


def frame_small(tag=""):
    """dock cam, grey 160x90. Returns (whole frame, tool crop), each zero-mean unit-norm. Saves the jpg for audit."""
    import io, urllib.request, numpy as np
    from PIL import Image
    raw = urllib.request.urlopen(R.URL + "/frame", timeout=5).read()
    if tag:
        os.makedirs(os.path.join(HERE, "refs", "wiggle"), exist_ok=True); open(os.path.join(HERE, "refs", "wiggle", tag + ".jpg"), "wb").write(raw)
    c = np.asarray(Image.open(io.BytesIO(raw)).convert("L").resize((160, 90)), float)
    def nz(x): x = x - x.mean(); return x / np.linalg.norm(x)
    r0, r1, c0, c1 = PROF.get("box", TOOL_BOX)
    return nz(c), nz(c[r0:r1, c0:c1])


def main(pick, unpick, n):
    res = []
    if "--fresh-refs" in sys.argv:                    # 11/01e: tool docked now -> docked ref; empty ref after 1st pick
        R.cmd("tool_off"); R.main("noop")             # starts the camera take (frames are live only in a take)
        time.sleep(2); tag = time.strftime("s%m%d_%H%M"); os.environ["TC_REFTAG"] = tag; snap("docked", tag); print("session refs", tag)
    t_start = time.time()
    for k in range(n):
        if "--max-min" in sys.argv and time.time() - t_start > 60 * float(arg("--max-min", "90")):
            print(f"time cap {arg('--max-min', '90')} min reached after {k} cycles"); break
        if "--rest-above" in sys.argv and k > 0:          # 11/02 eve: short rests between cycles instead of long cool-downs
            lim, t0 = float(arg("--rest-above", "48")), time.time()
            while R.get("/state")["temp"] > lim and time.time() - t0 < 1200: time.sleep(10)
            if time.time() - t0 > 5: print(f"rested {time.time() - t0:.0f} s to <= {lim} C")
        R.cmd("tool_off"); a = R.main(nm(pick, k))
        if a and "SHORT" in str(a.get("msg")) and os.path.exists(os.path.join(HERE, "paths", pick + "_retry.json")):
            print("short push, backed off; retry once from the lineup:", a.get("msg"))   # 11/02: the 1st contact nudges the tool home
            a0 = a; score("pick", a0, False, "na", "short_push", k, str(a0.get("msg"))[:100])   # pm-eng: first try counts
            fail_id = _streak["row"]; a = R.main(nm(pick + "_retry", k)); a["retry_of"] = fail_id
        row = {"k": k, "pick": [a.get("secs"), a.get("dock"), a.get("abort"), a.get("msg")], "retry": a.get("retry_of")}
        if k == 0 and "--fresh-refs" in sys.argv and a and not a.get("abort") and "done" in str(a.get("msg")):
            lab = R.dock_label()                          # empty ref missing: docked corr must have collapsed
            try: stall = int(str(a.get("msg")).rsplit(" ", 1)[-1])
            except ValueError: stall = None
            print("first pick: docked corr", lab[2], "stall", stall)
            if lab[2] < 0.5 and stall is not None and 2520 <= stall <= 2700:
                snap("empty", os.environ["TC_REFTAG"]); a["dock"] = ("empty", 1.0, lab[2]); print("empty ref taken, docked corr", lab[2])
        if not ok(a, "empty"):
            score("pick", a, False, "na", fmode(a), k, f"retry_of:{a.get('retry_of')}" if a.get("retry_of") else "", retry=bool(a.get("retry_of")))
            res.append(row); print("STOP after pick", row); break
        R.cmd("tool_on")
        if "--wiggle" in sys.argv:                        # 11/02 operator rule: a lock counts only after a wiggle test
            wt = f"{RUN_ID}_k{k}"; R.main(nm(PROF["settle"], k)); pa = R.get("/state")      # 11/02: settle AT the wiggle home first
            (fa, ca) = frame_small(wt + "_a"); wg = R.main(nm(PROF["wiggle"], k)); (fb, cb) = frame_small(wt + "_b"); pb = R.get("/state")
            corr = float((fa * fb).sum()); crop = float((ca * cb).sum()); held = R.dock_label()[0] != "docked"
            back = max(abs(pa[q] - pb[q]) for q in "xyz")
            row["wiggle"] = {"corr": round(corr, 3), "crop": round(crop, 3), "back_mm": round(back, 2), "abort": wg.get("abort"),
                             "end": wg.get("end"), "vmin": wg.get("vmin"), "temp": wg.get("temp")}
            print(f"wiggle: tool-area corr {crop:.3f} (>= {CROP_MIN}), whole frame {corr:.3f}, returned within {back:.1f} mm, tool not back in dock {held}, abort {wg.get('abort')}")
            if back > 4: print(f"WARNING wiggle return {back:.1f} mm (friction hysteresis is ~2 mm)")
            if wg.get("abort") or "done" not in str(wg.get("msg")) or crop < CROP_MIN or not held:
                row["wiggle"]["pass"] = False
                score("pick", a, False, "fail", f"wiggle crop {crop:.3f} frame {corr:.3f} back {back:.1f}mm held {held} abort {wg.get('abort')}", k)
                res.append(row); print("STOP: wiggle failed", row["wiggle"]); break
            row["wiggle"]["pass"] = True
        score("pick", a, True, "pass" if "--wiggle" in sys.argv else "na", "", k,
              (f"retry_of:{a.get('retry_of')}" if a.get("retry_of") else "") + (f"; wiggle crop {row['wiggle']['crop']} frame {row['wiggle']['corr']}" if row.get("wiggle") else ""),
              retry=bool(a.get("retry_of")))
        b = R.main(nm(unpick, k))
        row["unpick"] = [b.get("secs"), b.get("dock"), b.get("abort"), b.get("msg")]; res.append(row)
        if not ok(b, "docked"):
            score("putback", b, False, "na", fmode(b), k)
            print("STOP after put-back", row); break
        score("putback", b, True, "na", "", k)
        print(f"cycle {k}: CLEAN{' + WIGGLE' if row.get('wiggle') else ''} pick {a['secs']} s + put-back {b['secs']} s, temp {b['temp']} C, vmin {min(a.get('vmin') or 99, b.get('vmin') or 99)} V")
        if b["temp"] >= 50:
            if "--cool" not in sys.argv:
                print("motors >= 50 C: stopping to cool"); break
            t0 = time.time()                              # 11/02: wait at ready (holding) until <= COOL_TO, max 20 min
            while time.time() - t0 < 1200:
                time.sleep(15)
                try: tc = R.get("/state")["temp"]
                except Exception as ex: print("controller unreachable while cooling:", ex); raise SystemExit(2)
                if tc <= COOL_TO: break
            print(f"cooled to {tc} C in {time.time() - t0:.0f} s")
            row["cool"] = [round(time.time() - t0), tc]
    clean = sum(1 for r in res if "unpick" in r and r["unpick"][1] and r["unpick"][1][0] == "docked" and not r["unpick"][2]
                and (r.get("wiggle") or {}).get("pass", "--wiggle" not in sys.argv))
    with open(os.path.join(HERE, "logs", "cycle_runs.jsonl"), "a") as f:
        f.write(json.dumps({"t": time.strftime("%Y-%m-%d %H:%M:%S"), "pick": pick, "unpick": unpick, "clean": clean, "rows": res}) + "\n")
    print(f"CLEAN {clean}/{len(res)}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], int(sys.argv[3]))

# SPDX-License-Identifier: GPL-3.0-or-later
"""Program library for the tool-changer cycle (11/02, operator: save every version, track how long it took, replay + compare).

A PROGRAM = frozen copies of a pick path (+ its _retry) and a put-back path, plus the controller version it was run on.
Every cycle run through here is logged per cycle with per-step timings, so old and new programs compare on equal terms.

    python programs.py save P02 pick_f1 unpick_f1 --note "chain ctl, blended free moves"   freeze paths -> programs/P02/
    python programs.py run P02 3 [--wiggle --fresh-refs ...]   replay (materialises paths/prog_P02_*.json, runs cycle.py)
    python programs.py backfill P01 "2026-11-02 14:30" "2026-11-02 16:15"    ingest old runs of P01's paths in a window
    python programs.py report                                  -> ../PROGRAMS.html (per-program times, success, step bars)

Files: programs/registry.json, programs/<id>/{pick,pick_retry,unpick}.json, programs/runs.jsonl (one row per cycle).
ctl: "stutter" = before the 11/02 19:29 Ctl chain fix (moves ran ~1.33x slow with a still tick every 0.1 s), "chain" = after.
"""
import glob, hashlib, html, json, os, statistics as st, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
PD, PATHS, LOGS = os.path.join(HERE, "programs"), os.path.join(HERE, "paths"), os.path.join(HERE, "logs")
REG, RUNS = os.path.join(PD, "registry.json"), os.path.join(PD, "runs.jsonl")
os.makedirs(PD, exist_ok=True)


def reg():
    return json.load(open(REG)) if os.path.exists(REG) else {}


def ctl_version():
    src = open(os.path.join(HERE, "nudge.py")).read()
    v = "chain" if "11/02 chain" in src else "stutter"
    tune = os.path.join(HERE, "ctl_tune.json")
    if "FF_TAU" in src and os.path.exists(tune):
        ff = json.load(open(tune)).get("ff_tau", [0, 0, 0, 0])
        if any(ff): v += "+ff" + "/".join(f"{x:g}" for x in ff)
    return v, hashlib.sha1(src.encode()).hexdigest()[:10]


def save(pid, pick, unpick, note=""):
    r = reg(); d = os.path.join(PD, pid)
    if pid in r: raise SystemExit(f"{pid} exists: programs are frozen, save a new id")
    os.makedirs(d)
    files = {"pick": pick, "unpick": unpick}
    if os.path.exists(os.path.join(PATHS, pick + "_retry.json")): files["pick_retry"] = pick + "_retry"
    for k, n in files.items():
        open(os.path.join(d, k + ".json"), "w").write(open(os.path.join(PATHS, n + ".json")).read())
    r[pid] = {"pick": pick, "unpick": unpick, "saved": time.strftime("%Y-%m-%d %H:%M:%S"), "note": note,
              "ctl_at_save": ctl_version()[0], "steps": {k: len(json.load(open(os.path.join(d, k + ".json")))) for k in files}}
    json.dump(r, open(REG, "w"), indent=1); print("saved", pid, r[pid])


def materialise(pid):
    d = os.path.join(PD, pid)
    for k in ("pick", "pick_retry", "unpick"):
        f = os.path.join(d, k + ".json")
        if os.path.exists(f): open(os.path.join(PATHS, f"prog_{pid}_{k}.json"), "w").write(open(f).read())
    return f"prog_{pid}_pick", f"prog_{pid}_unpick"


def step_times(trace_file, steps):
    """trace rows [t, x, y, z, az, roll, loads, temp, 'name i/n'] -> [[i, kind, secs], ...] (time from a step's first row to the next's)"""
    try: tr = json.load(open(trace_file))
    except Exception: return []
    marks = []
    for row in tr:
        lab = str(row[-1]).rsplit(" ", 1)[-1]
        if "/" in lab and (not marks or marks[-1][0] != lab): marks.append((lab, row[0]))
    end = tr[-1][0] if tr else 0; out = []
    for j, (lab, t0) in enumerate(marks):
        i = int(lab.split("/")[0]) - 1; t1 = marks[j + 1][1] if j + 1 < len(marks) else end
        kind = next((k for k in ("via", "to", "joint", "twist_lock", "roll_from_stall", "roll", "expect", "qtraj", "dwell") if k in steps[i]), "?") if i < len(steps) else "?"
        out.append([i, kind, round(t1 - t0, 2)])
    return out


def path_runs_since(t0, t1="9999"):
    out = []
    for l in open(os.path.join(LOGS, "path_runs.jsonl")):
        try: d = json.loads(l)
        except ValueError: continue
        if t0 <= d.get("t", "") <= t1: out.append(d)
    return out


def trace_for(run):
    """trace file written at the end of this path run (name + stamp within ~2 min after start)"""
    name = run["path"].split("@")[0]; t = time.mktime(time.strptime(run["t"], "%Y-%m-%d %H:%M:%S"))
    best = None
    for f in glob.glob(os.path.join(LOGS, f"path_trace_{name}_*.json")):
        try: ts = time.mktime(time.strptime(f[-20:-5], "%Y%m%d_%H%M%S"))
        except ValueError: continue
        if 0 <= ts - t <= 240 and (best is None or ts < best[0]): best = (ts, f)
    return best[1] if best else None


def spikes_for(t0, t1):
    """temp_spike events in [t0, t1] -> {"n": count, "servo": {id: count}} (11/02 eve: find a flaky sensor/cable)"""
    n, by = 0, {}
    for l in open(os.path.join(LOGS, "nudge_events.jsonl")):
        try: d = json.loads(l)
        except ValueError: continue
        if d.get("kind") == "temp_spike" and t0 <= str(d.get("t", "")) <= t1:
            n += 1
            for i, (r, c) in enumerate(zip(d["raw"], d["confirmed"])):
                if r - c >= 8: by[i + 1] = by.get(i + 1, 0) + 1
    return {"n": n, "servo": by}


def twist_for(t0, t1):
    out = []
    for l in open(os.path.join(LOGS, "nudge_events.jsonl")):
        try: d = json.loads(l)
        except ValueError: continue
        if d.get("kind") == "twist-lock" and t0 <= d.get("t", "") <= t1: out.append(d.get("tries"))
    return out


def ingest(pid, t0, t1, ctl, source):
    """pair pick/put-back path runs of program pid in [t0, t1] into per-cycle rows"""
    p = reg()[pid]; d = os.path.join(PD, pid)
    pick_names = {p["pick"], p["pick"] + "_retry", f"prog_{pid}_pick", f"prog_{pid}_pick_retry"}
    unpick_names = {p["unpick"], f"prog_{pid}_unpick"}
    steps = {k: json.load(open(os.path.join(d, k + ".json"))) for k in ("pick", "unpick")}
    rows, cur = [], None
    for r in path_runs_since(t0, t1):
        n = r["path"].split("@")[0]
        if n in pick_names:
            if cur and cur.get("pick") and "retry" not in n: rows.append(cur); cur = None
            cur = cur or {"program": pid, "ctl": ctl, "source": source, "t": r["t"], "pick_tries": []}
            tf = trace_for(r)
            cur["pick_tries"].append({"path": r["path"], "secs": r["secs"], "msg": r.get("msg"), "abort": r.get("abort"),
                                      "steps": step_times(tf, steps["pick"] if "retry" not in n else []) if tf else []})
            cur["pick"] = r; cur["take"] = r.get("take")
        elif n in unpick_names and cur:
            tf = trace_for(r)
            cur["unpick"] = r; cur["unpick_steps"] = step_times(tf, steps["unpick"]) if tf else []
            rows.append(cur); cur = None
        elif cur and n.startswith(("wig", "wiggle")):
            cur["wiggle_s"] = round(cur.get("wiggle_s", 0) + r["secs"], 1)
    if cur: rows.append(cur)
    out = []
    for c in rows:
        pk, up = c.get("pick", {}), c.get("unpick", {})
        stall_ok = "last stall" in str(pk.get("msg")) and not pk.get("abort")
        try: stall = int(str(pk.get("msg")).rsplit(" ", 1)[-1])
        except ValueError: stall = None
        e = {"program": pid, "ctl": ctl, "source": source, "t": c["t"], "take": c.get("take"),
             "pick_s": round(sum(x["secs"] for x in c["pick_tries"]), 1), "pick_tries": len(c["pick_tries"]),
             "wiggle_s": c.get("wiggle_s"), "putback_s": up.get("secs"),
             "stall": stall, "lock_ok": bool(stall and 2520 <= stall <= 2650),
             "picked": stall_ok, "put_back": bool(up) and "done" in str(up.get("msg")) and not up.get("abort"),
             "temp": max(pk.get("temp") or 0, up.get("temp") or 0), "vmin": min(pk.get("vmin") or 99, up.get("vmin") or 99),
             "peak_elbow": max((pk.get("peaks") or {}).get("elbow", 0), (up.get("peaks") or {}).get("elbow", 0)),
             "twist": twist_for(c["t"], up.get("t", "9999") if up else "9999")[:1],
             "temp_spikes": spikes_for(c["t"], (up.get("t") if up else None) or "9999"),
             "pick_steps": c["pick_tries"][0]["steps"], "unpick_steps": c.get("unpick_steps", [])}
        e["cycle_s"] = round(e["pick_s"] + (e["putback_s"] or 0), 1) if e["putback_s"] else None
        out.append(e)
    with open(RUNS, "a") as f:
        for e in out: f.write(json.dumps(e) + "\n")
    for e in out: print(f"{e['t']} {pid}: pick {e['pick_s']} s ({e['pick_tries']} tries) + put-back {e['putback_s']} s = {e['cycle_s']} s, stall {e['stall']} lock_ok {e['lock_ok']}")
    return out


def run(pid, n, extra):
    pick, unpick = materialise(pid); t0 = time.strftime("%Y-%m-%d %H:%M:%S")
    rc = subprocess.call([sys.executable, "-u", os.path.join(HERE, "cycle.py"), pick, unpick, str(n), *extra], cwd=HERE)
    ingest(pid, t0, "9999", ctl_version()[0], "programs.py run"); report(); return rc


def report():
    r = reg(); runs = [json.loads(l) for l in open(RUNS)] if os.path.exists(RUNS) else []
    def med(v):
        v = [x for x in v if x]; return round(st.median(v), 1) if v else "–"
    def mn(v):
        v = [x for x in v if x]; return round(min(v), 1) if v else "–"
    trs = []
    for pid, p in r.items():
        rs = [x for x in runs if x["program"] == pid]
        for ctl in sorted({x["ctl"] for x in rs}) or ["–"]:
            q = [x for x in rs if x["ctl"] == ctl]; full = [x for x in q if x["cycle_s"]]
            trs.append(f"<tr><td><b>{pid}</b></td><td>{html.escape(p['pick'])} + {html.escape(p['unpick'])}</td><td>{ctl}</td><td>{len(q)}</td>"
                       f"<td>{sum(1 for x in q if x['picked'] and x['put_back'])}/{len(q)}</td><td>{sum(1 for x in q if x['lock_ok'])}/{len(q)}</td>"
                       f"<td>{med([x['pick_s'] for x in q])}</td><td>{med([x['putback_s'] for x in q])}</td><td>{med([x['cycle_s'] for x in full])}</td>"
                       f"<td>{mn([x['cycle_s'] for x in full])}</td><td>{html.escape(p.get('note', ''))}</td></tr>")
    # per-step bars: median seconds per step for each program/ctl's pick and put-back
    bars = []
    for pid in r:
        for ctl in sorted({x["ctl"] for x in runs if x["program"] == pid}):
            q = [x for x in runs if x["program"] == pid and x["ctl"] == ctl]
            for part in ("pick_steps", "unpick_steps"):
                by = {}
                for x in q:
                    for i, k, s in x[part]: by.setdefault((i, k), []).append(s)
                if not by: continue
                segs = [(i, k, st.median(v)) for (i, k), v in sorted(by.items())]; tot = sum(s for _, _, s in segs) or 1
                cells = "".join(f'<div class="seg k-{k}" style="flex:{s:.2f}" title="step {i} {k}: {s:.1f} s">{s:.0f}</div>' for i, k, s in segs)
                bars.append(f'<div class="row"><div class="lab">{pid} {ctl} {part.split("_")[0]} <span>{tot:.1f} s</span></div><div class="bar" style="width:{min(100, tot / 0.7):.0f}%">{cells}</div></div>')
    last = "".join(f"<tr><td>{x['t']}</td><td>{x['program']}</td><td>{x['ctl']}</td><td>{x['pick_s']}</td><td>{x['pick_tries']}</td><td>{x['wiggle_s'] or ''}</td>"
                   f"<td>{x['putback_s'] or ''}</td><td>{x['cycle_s'] or ''}</td><td>{x['stall']}</td><td>{'yes' if x['lock_ok'] else 'NO'}</td>"
                   f"<td>{x['temp']}</td><td>{x['vmin']}</td><td>{x['peak_elbow']}</td><td>{x['take'] or ''}</td></tr>" for x in reversed(runs[-60:]))
    page = f"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Tool Changer Programs</title><style>
:root{{--bg:#fbfaf7;--fg:#1d1d1b;--mut:#6b6a66;--line:#e2e0da;--via:#3a7bd5;--to:#8a8f98;--tw:#d9822b;--ro:#b5651d;--jo:#5a9e6f;--ex:#9b59b6}}
@media (prefers-color-scheme:dark){{:root:not([data-theme="light"]){{--bg:#181816;--fg:#ecebe6;--mut:#9a9893;--line:#33322f}}}}
:root[data-theme="dark"]{{--bg:#181816;--fg:#ecebe6;--mut:#9a9893;--line:#33322f}}
body{{background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,sans-serif;margin:0;padding:24px 16px;max-width:1100px;margin:auto}}
h1{{font-size:22px;margin:0 0 4px}} p{{color:var(--mut);margin:0 0 18px}} h2{{font-size:16px;margin:28px 0 8px}}
.tw{{overflow-x:auto}} table{{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}} td,th{{border-bottom:1px solid var(--line);padding:5px 8px;text-align:left;white-space:nowrap}} th{{color:var(--mut);font-weight:500}}
.row{{margin:6px 0}} .lab{{font-size:12px;color:var(--mut)}} .lab span{{color:var(--fg)}} .bar{{display:flex;gap:2px;height:22px;min-width:120px}}
.seg{{border-radius:3px;color:#fff;font-size:11px;display:flex;align-items:center;justify-content:center;overflow:hidden;background:var(--to)}}
.k-via{{background:var(--via)}} .k-twist_lock{{background:var(--tw)}} .k-roll_from_stall,.k-roll{{background:var(--ro)}} .k-joint{{background:var(--jo)}} .k-expect{{background:var(--ex)}}
.key span{{display:inline-block;padding:1px 6px;border-radius:3px;color:#fff;font-size:11px;margin-right:4px}}
</style></head><body><h1>Tool changer programs</h1>
<p>Every saved program and every cycle run through <code>programs.py</code>. Times in seconds. ctl "stutter" = old controller (moves ~1.33&times; slow), "chain" = fixed 11/02. Lock ok = twist stall 2520–2650. Updated {time.strftime('%Y-%m-%d %H:%M')}.</p>
<h2>Programs</h2><div class="tw"><table><tr><th>id</th><th>paths</th><th>ctl</th><th>runs</th><th>clean</th><th>lock ok</th><th>pick med</th><th>put-back med</th><th>cycle med</th><th>cycle best</th><th>note</th></tr>{''.join(trs)}</table></div>
<h2>Where the time goes (median per step)</h2><div class="key"><span class="k-joint seg" style="display:inline-block">joint</span><span class="k-via seg" style="display:inline-block">via (blended)</span><span class="seg" style="display:inline-block">to (stop at end)</span><span class="k-expect seg" style="display:inline-block">expect</span><span class="k-twist_lock seg" style="display:inline-block">twist lock</span><span class="k-roll seg" style="display:inline-block">roll</span></div>{''.join(bars)}
<h2>Runs (latest first)</h2><div class="tw"><table><tr><th>time</th><th>prog</th><th>ctl</th><th>pick</th><th>tries</th><th>wiggle</th><th>put-back</th><th>cycle</th><th>stall</th><th>lock ok</th><th>&deg;C</th><th>V min</th><th>elbow %</th><th>take</th></tr>{last}</table></div>
</body></html>"""
    out = os.path.join(HERE, "..", "PROGRAMS.html"); open(out, "w", encoding="utf-8").write(page); print("report", os.path.abspath(out))


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[0] == "save": save(a[1], a[2], a[3], a[a.index("--note") + 1] if "--note" in a else "")
    elif a[0] == "run": sys.exit(run(a[1], int(a[2]), a[3:]))
    elif a[0] == "backfill": ingest(a[1], a[2], a[3], a[4] if len(a) > 4 else "stutter", "backfill")
    elif a[0] == "report": report()

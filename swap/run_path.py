# SPDX-License-Identifier: GPL-3.0-or-later
"""Run a keyframe path (paths/<name>.json) on nudge_web with a 10 Hz load/temp watchdog that presses STOP.
    python run_path.py <name>
Logs: logs/path_runs.jsonl (summary) + logs/path_trace_<name>_<stamp>.json (pose/roll/loads/temp at ~10 Hz)."""
import json, os, sys, threading, time, urllib.request

URL = os.environ.get("TC_URL", "http://127.0.0.1:8765"); HERE = os.path.dirname(os.path.abspath(__file__))
LIM = {"pan": 60, "lift": 60, "elbow": 75, "flex": 60, "roll": 101}  # roll stalls on purpose (twist to stall, time-capped in the path)
TEMP_MAX = 55
VOLT_MIN = 10.8                     # 11/02: bus volts (min of lift/elbow) below this twice in a row = stop


def get(p): return json.loads(urllib.request.urlopen(URL + p, timeout=3).read())
def cmd(c, a=""): urllib.request.urlopen(f"{URL}/cmd?c={c}&a={a}", timeout=3).read()


def dock_label():
    """Dock cam patch (tool's spot on the post) vs refs/dock_{empty,docked}_1101.jpg. 11/01: 0.92-1.00 vs 0.12-0.16."""
    import io, numpy as np
    from PIL import Image
    def crop(img):
        c = np.asarray(img.convert("L"), float)[215:355, 200:265]; c = c - c.mean(); return c / np.linalg.norm(c)
    raw = urllib.request.urlopen(URL + "/frame", timeout=3).read(); time.sleep(0.6)
    raw2 = urllib.request.urlopen(URL + "/frame", timeout=3).read()
    import camcheck                                             # 11/02 eve: OBS placeholder read as "docked 1.0"
    bad = camcheck.frame_problem(raw2, raw)
    if bad: return "camera_invalid", 0.0, 0.0, bad
    fr = crop(Image.open(io.BytesIO(raw2)))
    tag = os.environ.get("TC_REFTAG", "1101")                  # 11/01e: per-session refs (cycle.py), lights/cam move
    ce = float((fr * crop(Image.open(os.path.join(HERE, "refs", f"dock_empty_{tag}.jpg")))).sum())         if os.path.exists(os.path.join(HERE, "refs", f"dock_empty_{tag}.jpg")) else 0.0
    cd = float((fr * crop(Image.open(os.path.join(HERE, "refs", f"dock_docked_{tag}.jpg")))).sum())
    lab = "docked" if cd > 0.6 and cd > ce + 0.3 else "empty" if ce > 0.6 and ce > cd + 0.3 else "unsure"
    return lab, round(ce, 2), round(cd, 2)


def main(name):
    s = get("/state"); assert not s["frozen"] and not s["busy"], s
    log = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "path": name, "start": [s[k] for k in "x y z az roll".split()],
           "tool_on_before": s["tool_on"], "abort": None, "kit": s.get("kit")}
    peaks = {k: 0.0 for k in LIM}; trace = []; t0 = time.time()
    cmd("path", name); time.sleep(0.5)
    fails = 0; st = s; hot = 0; low = 0; log["vmin"] = s.get("volt")
    while True:
        try:
            st = get("/state"); fails = 0
        except Exception as ex:                 # state() reads the bus: a power cut-out shows up here
            fails += 1
            if fails >= 10:
                log["abort"] = f"BUS LOST (12 V cut-out?): {ex}"; st = dict(s, busy="", msg="bus lost", take=None); break
            time.sleep(0.1); continue
        trace.append([round(time.time() - t0, 2), st["x"], st["y"], st["z"], st["az"], st["roll"], st["load"], st["temp"], st["busy"]])
        for k, v in st["load"].items():
            peaks[k] = max(peaks[k], abs(v))
            if abs(v) > LIM[k] and not log["abort"]:
                cmd("stop"); log["abort"] = f"{k} load {v}% at {st['busy']}"
        hot = hot + 1 if st["temp"] >= TEMP_MAX else 0          # 11/01e: one-sample 37->55->37 glitch stopped a run
        if hot >= 4 and not log["abort"]:                         # 11/02: 35->67->35 double-read glitch; need 0.4 s of heat
            cmd("stop"); log["abort"] = f"temp {st['temp']} C"
        v = st.get("volt")                                         # 11/02: 12 V has cut out 3x; log the min, stop on a dip
        if v is not None:
            log["vmin"] = v if log["vmin"] is None else min(log["vmin"], v)
            low = low + 1 if v < VOLT_MIN else 0
            if low >= 2 and not log["abort"]:
                cmd("stop"); log["abort"] = f"12 V dip: {v} V"
        if not st["busy"]: break
        time.sleep(0.1)
    log.update(secs=round(time.time() - t0, 1), end=[st[k] for k in "x y z az roll".split()], end_load=st["load"],
               temp=st["temp"], peaks=peaks, msg=st["msg"], take=st["take"])
    time.sleep(1.5)
    try: log["dock"] = dock_label()
    except Exception as ex: log["dock"] = f"err {ex}"
    with open(os.path.join(HERE, "logs", "path_runs.jsonl"), "a") as f: f.write(json.dumps(log) + "\n")
    json.dump(trace, open(os.path.join(HERE, "logs", f"path_trace_{name}_{time.strftime('%Y%m%d_%H%M%S')}.json"), "w"))
    print(json.dumps(log)); return log


if __name__ == "__main__":
    main(sys.argv[1])

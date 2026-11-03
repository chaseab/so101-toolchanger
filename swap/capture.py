# SPDX-License-Identifier: GPL-3.0-or-later
"""Capture setup for the first automatic swap (reel SX6 + test record).  Software only: it never touches the arm.

  python capture.py check [--shots]      cameras found? disk? one still per camera -> framing_<cam>.jpg + framing_sheet.jpg
  python capture.py soak [--secs 60]     record all cameras at once for N s, count dropped/short streams, delete the files
  python capture.py start [--term TITLE] start every camera (+ terminal grab) NOW, 5 s pre-roll is the time before you
                                         start run_swap; leaves them running; writes takes.json
  python capture.py mark <event>         append a wall-clock event (approach/seat/latch/lift-away/result/...) to takes.json
  python capture.py stop                 5 s post-roll, then stop every recorder politely (q), final takes.json
  python capture.py events <swap_log.csv> add the run's own phase changes (from the swap CSV log) to takes.json

Cameras: wrist = the Innomaker whose USB parent reports serial SN0001; scene = the other Innomaker (the overhead/bench cam,
or the dock cam once it is moved onto the dock mount - same module either way).  A third UVC camera (e.g. an OV9732) is
picked up as `dock` if it shows up.  Output 1080p30 H.264 (libx264 ultrafast crf 20, never MJPEG copy), one file per cam per
run: tc_<cam>_1028_<HHMMSS>.mkv in OUT (capture_dir in config.json).  Guard: stops everything if its disk has < 12 GB free.
No burned timecode on the masters: takes.json holds the wall-clock start of every file.
"""
import argparse, json, os, re, shutil, signal, subprocess, sys, time
from pathlib import Path

import tc_config  # noqa: E402  (machine-specific settings: config.json, see config.example.json)
OUT = Path(tc_config.CAPTURE_DIR)
STATE = OUT / "capture_state.json"
TAKES = OUT / "takes.json"
GUARD_GB = 12.0
WRIST_SERIAL = "SN0001"
FF = "ffmpeg"


def free_gb(path=None) -> float:
    return shutil.disk_usage(str(path or OUT)).free / 2 ** 30


def free_ram_gb() -> float:
    out = subprocess.run(["powershell", "-NoProfile", "-Command", "(Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory"], capture_output=True, text=True, timeout=30).stdout.strip()
    return float(out) / 2 ** 20 if out else 0.0     # KB -> GB


def preflight() -> bool:
    """pm-eng rule: free RAM >= 8 GB and C: >= 15 GB right before capture, else do not start (message pm-eng + pm-misc and wait)."""
    r, d = free_ram_gb(), free_gb()
    ok = r >= 8.0 and d >= 17.0
    print(f"preflight: free RAM {r:.1f} GB (need >= 8), C: {d:.1f} GB (need >= 17) -> {'OK' if ok else 'NOT STARTING: message pm-eng and pm-misc, wait for them to free resources'}")
    return ok


def ps(cmd: str) -> str:
    return subprocess.run(["powershell", "-NoProfile", "-Command", cmd], capture_output=True, text=True, timeout=60).stdout


def find_cameras() -> dict:
    """{name: dshow alternative-name} for every UVC camera we record."""
    raw = subprocess.run([FF, "-hide_banner", "-list_devices", "true", "-f", "dshow", "-i", "dummy"],
                         capture_output=True, text=True).stderr
    devs, last = [], None
    for ln in raw.splitlines():
        m = re.search(r'"([^"]+)" \(video\)', ln)
        if m:
            last = {"name": m.group(1), "alt": None}
            devs.append(last)
            continue
        m = re.search(r'Alternative name "([^"]+)"', ln)
        if m and last is not None and last["alt"] is None:
            last["alt"] = m.group(1)
    # USB children -> parent serial, so "wrist" is identified by SN0001 and not by enumeration order
    rows = ps("Get-PnpDevice -Class Camera -PresentOnly | ?{$_.InstanceId -like 'USB*'} | %{ "
              "$p=(Get-PnpDeviceProperty -InstanceId $_.InstanceId -KeyName DEVPKEY_Device_Parent).Data; "
              "\"$($_.InstanceId)|$p\" }").strip().splitlines()
    cams = {}
    for d in devs:
        if d["alt"] is None or d["name"].startswith(("EOS", "OBS")):
            continue
        tag = re.search(r"mi_00#([0-9a-f&]+)#", d["alt"], re.I)
        parent = ""
        for r in rows:
            iid, _, par = r.partition("|")
            if tag and tag.group(1).upper() in iid.upper():
                parent = par
        if WRIST_SERIAL in parent:
            cams["wrist"] = d["alt"]
        elif d["name"].startswith("Innomaker") and "scene" not in cams:
            cams["scene"] = d["alt"]
        else:
            cams.setdefault("dock" if "dock" not in cams else f"extra{len(cams)}", d["alt"])
    if "scene" not in cams and "wrist" in cams:      # 10/31: wrist unplugged (its USB feeds the D: drive); the one
        cams["scene"] = cams.pop("wrist")            # Innomaker left reports SN0001 too -> it IS the scene/dock cam
    return cams


def cam_args(alt: str, size="1920x1080") -> list:
    return ["-f", "dshow", "-rtbufsize", "256M", "-vcodec", "mjpeg", "-video_size", size, "-framerate", "30",
            "-i", f"video={alt}"]


ENC = ["-c:v", "libx264", "-preset", "veryfast", "-crf", "24", "-maxrate", "8M", "-bufsize", "16M", "-pix_fmt", "yuv420p", "-threads", "2"]   # ~8 Mbps cap = <= 3.6 GB/h per camera


def cmd_check(a):
    OUT.mkdir(parents=True, exist_ok=True)
    cams = find_cameras()
    print(f"C: free {free_gb():.1f} GB (guard {GUARD_GB:.0f} GB; start needs >= 17 GB)")
    preflight()
    print("cameras:", {k: v[-40:] for k, v in cams.items()} or "NONE FOUND")
    if "wrist" not in cams or "scene" not in cams:
        print("PROBLEM: need both Innomakers (wrist SN0001 + scene)")
    if a.shots:
        from PIL import Image
        ims = []
        for name, alt in cams.items():
            f = OUT / f"framing_{name}.jpg"
            if name == "scene":                   # hero camera: locked exposure/gain/WB (same process settings the recording uses)
                idx = scene_cv_index(cams)
                import numpy as np
                from PIL import Image as _I
                for lad in LADDER:                 # auto-tune: first (brightest) setting with mean <= 130 and clipped highlights < 2 %
                    la = ["--exposure", str(lad[0]), "--gain", str(lad[1]), "--brightness", str(lad[2])]
                    r = subprocess.run([sys.executable, str(Path(__file__).with_name("capture_cv.py")), "--index", str(idx), "--still", str(f), *la],
                                       capture_output=True, text=True, timeout=60)
                    g_ = np.asarray(_I.open(f).convert("L"), float)
                    if g_.mean() <= 130 and (g_ > 250).mean() < 0.02:
                        break
                c_ = np.asarray(_I.open(f).convert("RGB"), float); L_ = c_.mean(2)          # WB gains from the bright near-white pixels
                m_ = (L_ > np.percentile(L_, 85)) & (L_ < 245); w_ = c_[m_].mean(0) if m_.any() else np.ones(3)
                gn = np.clip(w_[1] / w_, 0.7, 1.5); gn = gn / gn.max()                    # max gain 1: never clips highlights
                wb = [round(float(gn[0]), 3), round(float(gn[1]), 3), round(float(gn[2]), 3)]
                LOCK_JSON.write_text(json.dumps({"ladder": list(lad), "mean": round(float(g_.mean()), 1), "wb_rgb": wb}))
                subprocess.run([sys.executable, str(Path(__file__).with_name("capture_cv.py")), "--index", str(idx), "--still", str(f), *la,
                                "--wb-r", str(wb[0]), "--wb-g", str(wb[1]), "--wb-b", str(wb[2])], capture_output=True, text=True, timeout=60)
                print("white balance gains (r, g, b):", wb)
                print("auto-tuned lock (exposure, gain, brightness):", lad, "-> saved to", LOCK_JSON.name)
                print("scene (locked) via DSHOW index", idx, "->", (r.stdout or r.stderr).strip()[-160:])
            else:
                subprocess.run([FF, "-y", "-loglevel", "error", *cam_args(alt), "-frames:v", "20", "-update", "1", str(f)],
                               capture_output=True)   # 20 frames so auto-exposure settles; last one is kept
            print(("ok " if f.exists() else "FAILED ") + str(f))
            if f.exists():
                ims.append((name, Image.open(f)))
                if name == "scene":
                    import numpy as np
                    im = Image.open(f).convert("RGB"); g = np.asarray(im.convert("L"), float)
                    print(f"scene exposure: mean {g.mean():.0f}/255, clipped highlights {(g > 250).mean() * 100:.1f} %, crushed shadows {(g < 8).mean() * 100:.1f} %  (aim mean 70-130, clipped < 2 %)")
                    cw = int(im.height * 9 / 16)                      # phone check: 9:16 crop (default centre, --crop-x 0..1) scaled to 135x240
                    x0 = int((im.width - cw) * a.crop_x)
                    im.crop((x0, 0, x0 + cw, im.height)).resize((135, 240), Image.LANCZOS).save(OUT / "framing_scene_phone.png")
                    im.resize((427, 240), Image.LANCZOS).save(OUT / "framing_scene_240p.png")
                    print("phone-size previews:", OUT / "framing_scene_phone.png", "(9:16 crop at x=%.2f)" % a.crop_x, "and", OUT / "framing_scene_240p.png", "-> LOOK at them: dock + tool must read")
        if ims:
            from PIL import ImageDraw
            w = 640
            tiles = []
            for n, im in ims:
                t = im.resize((w, int(im.height * w / im.width)))
                ImageDraw.Draw(t).text((8, 8), n, fill=(255, 220, 0))
                tiles.append(t)
            sheet = Image.new("RGB", (w * len(tiles), max(t.height for t in tiles)))
            for i, t in enumerate(tiles):
                sheet.paste(t, (i * w, 0))
            sheet.save(OUT / "framing_sheet.jpg", quality=88)
            print("sheet", OUT / "framing_sheet.jpg")
    return 0


LOCK_JSON = OUT / "lock.json"          # chosen by `check --shots` (auto-tune), reused by start/leader while < 6 h old
LADDER = [(-5, 64, 32), (-5, 48, 28), (-5, 36, 24), (-5, 24, 16), (-5, 12, 8), (-6, 12, 8), (-6, 0, 0), (-7, 0, 0), (-8, 0, 0)]   # brightest -> dimmest (exposure, gain, brightness)


def lock_args():
    if os.environ.get("TC_EXPOSURE"):
        return ["--exposure", os.environ["TC_EXPOSURE"], "--gain", os.environ.get("TC_GAIN", "36"), "--brightness", os.environ.get("TC_BRIGHT", "24")]
    try:
        if LOCK_JSON.exists() and time.time() - LOCK_JSON.stat().st_mtime < 6 * 3600:
            j = json.loads(LOCK_JSON.read_text()); e, g, b = j["ladder"]; wr, wg, wb = j.get("wb_rgb", [1.0, 1.0, 1.0])
            return ["--exposure", str(e), "--gain", str(g), "--brightness", str(b), "--wb-r", str(wr), "--wb-g", str(wg), "--wb-b", str(wb)]
    except Exception:
        pass
    return ["--exposure", "-5", "--gain", "36", "--brightness", "24"]


LOCK_ARGS = lock_args()


def scene_cv_index(cams) -> int:
    """OpenCV DirectShow index of the scene camera: match one still against ffmpeg's frame of the scene device (the scene is static;
    the DSHOW order differs between ffmpeg and OpenCV and between boots, so never trust a fixed index)."""
    import cv2, numpy as np
    cache = OUT / "scene_idx.json"          # written by `check --shots`; `start` reuses it (saves ~5 s of startup) while it is < 6 h old
    sys.path.insert(0, str(Path(__file__).parent)); import camcheck   # 11/02 eve: candidates by DEVICE NAME, never OBS/EOS
    real = camcheck.real_indices()
    cands = real if real is not None else [1, 2]
    if not cands:
        raise SystemExit("REFUSED: no non-virtual camera found (names: %s)" % camcheck.dshow_names())
    if cache.exists() and time.time() - cache.stat().st_mtime < 6 * 3600 and not os.environ.get("TC_RECHECK_IDX"):
        ci = json.loads(cache.read_text())["index"]
        if ci in cands:
            return ci
    def norm(g):
        g = cv2.resize(g, (64, 36)).astype(float)
        return (g - g.mean()) / (g.std() + 1e-6)
    refs = {}
    for nm in [n for n in ("scene", "wrist") if n in cams]:     # 10/31: wrist may be unplugged
        p = OUT / f"_idx_ref_{nm}.jpg"
        subprocess.run([FF, "-y", "-loglevel", "error", *cam_args(cams[nm]), "-frames:v", "15", "-update", "1", str(p)], capture_output=True)
        refs[nm] = norm(cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2GRAY)); p.unlink(missing_ok=True)
    got = {}
    for i in cands:                       # 11/02 eve: real devices only (was (1, 2): picked OBS when 1 was busy)
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if not cap.isOpened():
            continue
        cap.set(cv2.CAP_PROP_FPS, 30); cap.set(3, 1920); cap.set(4, 1080); cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        for _ in range(12):
            cap.read()
        ok, f = cap.read(); cap.release()
        if ok:
            got[i] = norm(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
    # joint assignment: scene<->index a, wrist<->the other; pick the pairing with the smaller total normalised difference
    idxs = sorted(got)
    if not idxs:
        raise SystemExit(f"REFUSED: no real camera would open (candidates {cands}; one may be held by an orphan recorder)")
    best, bi = 1e9, idxs[0]
    for a_ in idxs:
        others = [k for k in idxs if k != a_]
        tot = abs(got[a_] - refs["scene"]).mean() + (abs(got[others[0]] - refs["wrist"]).mean() if others and "wrist" in refs else 0)
        if tot < best:
            best, bi = tot, a_
    cache.write_text(json.dumps({"index": bi, "pair_diff": round(float(best), 2)}))
    return bi


def spawn_cam(name, alt, stamp, cams, preview=False):
    """scene = locked-exposure OpenCV worker (capture_cv.py); every other camera = plain ffmpeg dshow.
    preview: the wrist goes through capture_cv.py too (auto exposure) so both can show a live window while recording."""
    if preview and name == "wrist":
        si = scene_cv_index(cams)
        f = OUT / f"tc_wrist_1028_{stamp}.mkv"
        stop = OUT / f"STOP_{stamp}"
        p = subprocess.Popen([sys.executable, str(Path(__file__).with_name("capture_cv.py")), "--index", str(3 - si), "--out", str(f),
                              "--stop-file", str(stop), "--auto", "--preview", "WRIST cam", "--preview-x", "970"],
                             creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
        return {"cam": "wrist", "file": f.name, "pid": p.pid, "t_start": time.time(), "proc": p, "stop_file": str(stop), "ready_file": str(f) + ".ready"}
    if name != "scene":
        return spawn(name, [*cam_args(alt), *ENC], stamp)
    f = OUT / f"tc_scene_1028_{stamp}.mkv"
    stop = OUT / f"STOP_{stamp}"
    stop.unlink(missing_ok=True)
    p = subprocess.Popen([sys.executable, str(Path(__file__).with_name("capture_cv.py")), "--index", str(scene_cv_index(cams)), "--out", str(f),
                          "--stop-file", str(stop), *LOCK_ARGS, *(["--preview", "DOCK cam"] if preview else [])], creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
    return {"cam": "scene", "file": f.name, "pid": p.pid, "t_start": time.time(), "proc": p, "stop_file": str(stop), "ready_file": str(f) + ".ready"}


def wait_ready(runs, secs=40):
    """The locked-exposure worker needs a few seconds to open the camera; wait until it writes its .ready file (so the 5 s pre-roll counts from when ALL cameras are live)."""
    t0 = time.time()
    for r in runs:
        rf = r.get("ready_file")
        while rf and not os.path.exists(rf) and time.time() - t0 < secs:
            time.sleep(0.5)
        if rf:
            print(f"{r['cam']}: {'ready' if os.path.exists(rf) else 'NOT READY after %d s' % secs} ({time.time() - t0:.1f} s)")


def spawn(name, args, stamp):
    f = OUT / f"tc_{name}_1028_{stamp}.mkv"
    log = open(OUT / f"tc_{name}_1028_{stamp}.log", "wb")
    live = [] if "-filter_complex" in args else ["-map", "0:v", "-vf", "fps=2,scale=800:-1", "-update", "1", "-y", str(f) + ".latest.jpg"]   # live frame (10/30)
    p = subprocess.Popen([FF, "-y", "-loglevel", "warning", *args, str(f), *live], stdin=subprocess.PIPE, stdout=log,
                         stderr=log, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
    return {"cam": name, "file": f.name, "pid": p.pid, "t_start": time.time(), "proc": p}


def cmd_soak(a):
    OUT.mkdir(parents=True, exist_ok=True)
    cams = find_cameras()
    stamp = "soak" + time.strftime("%H%M%S")
    runs = [spawn_cam(n, alt, stamp, cams, a.preview) for n, alt in cams.items()]
    wait_ready(runs)
    t0 = time.time()
    while time.time() - t0 < a.secs:
        time.sleep(1)
        if free_gb() < GUARD_GB:
            print("disk guard")
            break
    (OUT / f"STOP_{stamp}").write_text("stop")           # the locked scene worker polls this
    for r in runs:
        try:
            r["proc"].stdin.write(b"q"); r["proc"].stdin.flush()
        except Exception:
            pass
    for r in runs:
        try:
            r["proc"].wait(timeout=20)
        except Exception:
            r["proc"].kill()
    for r in runs:
        f = OUT / r["file"]
        dur = 0.0
        if f.exists():
            o = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(f)],
                               capture_output=True, text=True).stdout.strip()
            dur = float(o) if o else 0.0
        lg = (OUT / (r["file"] + ".log" if r["cam"] == "scene" else r["file"].replace(".mkv", ".log"))).read_text(errors="ignore")
        drops = len(re.findall(r"dropp|real-time buffer|overrun|error", lg, re.I))
        print(f"{r['cam']:6s} {dur:5.1f}s of {a.secs}s  size {f.stat().st_size / 1e6 if f.exists() else 0:6.0f} MB  warnings {drops}"
              f"  -> {'OK' if dur >= a.secs - 3 and drops == 0 else 'CHECK (fallback: that cam at 1280x720)'}")
        for x in (f, OUT / r["file"].replace(".mkv", ".log"), OUT / (r["file"] + ".log"), OUT / (r["file"] + ".ready")):
            x.unlink(missing_ok=True)
    return 0


def load():
    return json.loads(TAKES.read_text()) if TAKES.exists() else {"events": [], "files": []}


def save(d):
    TAKES.write_text(json.dumps(d, indent=1))


def cmd_start(a):
    OUT.mkdir(parents=True, exist_ok=True)
    if not preflight():
        return 2
    left = stop_recorders(15)                   # 11/02 eve: no new take while ANY recorder still runs (it holds the camera)
    if left:
        print("REFUSED: recorders still alive (pids %s), not starting a take" % left); return 2
    cams = find_cameras()
    if not cams:
        print("no cameras"); return 2
    stamp = time.strftime("%H%M%S")
    runs = [spawn_cam(n, alt, stamp, cams, a.preview) for n, alt in cams.items()]
    wait_ready(runs)
    if a.term:      # the run_swap terminal window only, 10 fps: proof that the arm ran by itself
        runs.append(spawn("term", ["-f", "gdigrab", "-framerate", "10", "-i", f"title={a.term}", *ENC[:6], "-pix_fmt", "yuv420p"],
                          stamp))
    if a.audio:
        runs.append(spawn("audio", ["-f", "dshow", "-i", "audio=Microphone (WO Mic Device)"], stamp))
        runs[-1]["file"] = runs[-1]["file"]
    d = load()
    d["run_stamp"] = stamp
    for r in runs:
        d["files"].append({k: r[k] for k in ("cam", "file", "pid", "t_start")})
    d["events"].append({"t": time.time(), "event": "capture_started"})
    save(d)
    STATE.write_text(json.dumps({"pids": [r["pid"] for r in runs], "stamp": stamp}))
    print("recording:", ", ".join(r["file"] for r in runs))
    print("wait ~5 s (pre-roll), then start run_swap; `capture.py mark approach|seat|latch|lift-away|result`; `capture.py stop` after")
    # guard: detached watcher stops everything if C: drops below the limit (it exits when the recorders do)
    subprocess.Popen([sys.executable, __file__, "guard"], creationflags=subprocess.CREATE_NEW_PROCESS_GROUP | 0x08000000)
    return 0


def cmd_guard(a):
    while True:
        time.sleep(5)
        st = json.loads(STATE.read_text()) if STATE.exists() else {"pids": []}
        alive = [p for p in st["pids"] if ps(f"(Get-Process -Id {p} -ErrorAction SilentlyContinue).Id").strip()]
        if not alive:
            return 0
        if free_gb() < GUARD_GB:
            d = load(); d["events"].append({"t": time.time(), "event": f"DISK GUARD stop ({free_gb():.1f} GB free)"}); save(d)
            stop_all()
            (OUT / "DISK_GUARD_TRIPPED.txt").write_text(f"{time.strftime('%H:%M:%S')} C: free {free_gb():.1f} GB < {GUARD_GB} GB: ALL RECORDERS STOPPED\n")
            try:
                import winsound
                for _ in range(5):
                    winsound.Beep(1200, 400)
            except Exception:
                pass
            return 0


def live_recorders():
    """every capture_cv.py recorder process alive now: [(pid, stop_file)] (11/02 eve: an orphan held the dock cam)"""
    import re
    out = ps("Get-CimInstance Win32_Process -Filter \"Name like 'python%'\" | Where-Object { $_.CommandLine -match 'capture_cv.py' -and $_.CommandLine -match '--out' } "
             "| ForEach-Object { \"$($_.ProcessId)|$($_.CommandLine)\" }")
    res = []
    for line in out.splitlines():
        if "|" not in line: continue
        pid, cl = line.split("|", 1); m = re.search(r"--stop-file\s+(\S+)", cl)
        res.append((int(pid), m.group(1) if m else None))
    return res


def stop_recorders(secs=15):
    """write each live recorder's own stop file, wait for it to EXIT (confirmed); returns the pids still alive"""
    recs = live_recorders()
    for pid, sf in recs:
        if sf: Path(sf).write_text("stop")
    t0 = time.time()
    while time.time() - t0 < secs:
        alive = [p for p, _ in live_recorders()]
        if not alive: return []
        time.sleep(1)
    return [p for p, _ in live_recorders()]


def stop_all():
    st = json.loads(STATE.read_text()) if STATE.exists() else {"pids": []}
    if st.get("stamp"):
        (OUT / f"STOP_{st['stamp']}").write_text("stop")      # the locked-exposure worker polls this and closes its ffmpeg cleanly
        time.sleep(3)
    for pid in st["pids"]:                      # CTRL_BREAK lets ffmpeg finalise the mkv (stdin 'q' is not reachable from here)
        try:
            os.kill(pid, signal.CTRL_BREAK_EVENT)
        except Exception:
            pass
    time.sleep(4)
    for pid in st["pids"]:                      # only the PIDs WE recorded
        if ps(f"(Get-Process -Id {pid} -ErrorAction SilentlyContinue).Id").strip():
            subprocess.run(["taskkill", "/PID", str(pid), "/F"], capture_output=True)
    left = stop_recorders(10)                   # 11/02 eve: also any orphan recorder (state clobbered by a double start); confirm exit
    if left: print("WARNING: recorders still alive after stop:", left)


def cmd_mark(a):
    d = load(); d["events"].append({"t": time.time(), "event": a.event}); save(d)
    print(time.strftime("%H:%M:%S"), a.event)
    return 0


def cmd_stop(a):
    time.sleep(5)                               # post-roll
    d = load(); d["events"].append({"t": time.time(), "event": "capture_stop"}); save(d)
    stop_all()
    print("stopped; files in", OUT)
    for f in sorted(OUT.glob("tc_*_1028_*.mkv")):
        print(f"  {f.name}  {f.stat().st_size / 1e6:.0f} MB")
    return 0


def cmd_events(a):
    import csv
    rows = list(csv.DictReader(open(a.csv, newline="")))
    keys = [k for k in (rows[0].keys() if rows else []) if re.search(r"phase|label|state|seg|note|stage", k, re.I)]
    t_ctl = Path(a.csv).stat().st_ctime
    d = load()
    if keys:
        last = None
        for r in rows:
            v = r[keys[0]]
            if v != last:
                d["events"].append({"t_rel_csv": r.get("t") or r.get("time"), "event": f"swap:{v}", "csv": Path(a.csv).name})
                last = v
    # per-attempt numbers for the reel: coupling-face error at standoff/contact (mm) if the log has an error column, else n/a
    ek = [k for k in (rows[0].keys() if rows else []) if re.search(r"err|offset|resid|track", k, re.I)]
    cand = {}
    for k in ek:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[k]))
            except (TypeError, ValueError):
                pass
        if vals:
            cand[k] = {"max_abs": round(max(abs(v) for v in vals), 3), "last": vals[-1]}
    d["attempt"] = {"coupling_error_mm": cand or "n/a", "result": a.result or "n/a (pass --result 'pick ok|fail, drop ok|fail, lock check ...')",
                    "note": "error columns are whatever the swap CSV logs (units as logged); n/a if none"}
    d["swap_log"] = {"file": Path(a.csv).name, "columns": list(rows[0].keys()) if rows else [], "created": t_ctl}
    save(d)
    print(f"added {len(d['events'])} events total; csv columns: {list(rows[0].keys())[:12] if rows else '-'}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)
    c = sp.add_parser("check"); c.add_argument("--shots", action="store_true"); c.add_argument("--crop-x", type=float, default=0.5)
    c = sp.add_parser("soak"); c.add_argument("--secs", type=int, default=60)
    c = sp.add_parser("start"); c.add_argument("--term", default=""); c.add_argument("--audio", action="store_true"); c.add_argument("--preview", action="store_true")
    sp.add_parser("guard")
    c = sp.add_parser("mark"); c.add_argument("event")
    sp.add_parser("stop")
    c = sp.add_parser("events"); c.add_argument("csv"); c.add_argument("--result", default="")
    a = ap.parse_args()
    sys.exit(globals()["cmd_" + a.cmd](a))

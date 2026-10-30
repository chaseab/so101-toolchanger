# SPDX-License-Identifier: GPL-3.0-or-later
"""Locked-exposure recorder for ONE UVC camera (the dock/scene cam): OpenCV DirectShow with manual exposure / gain / brightness /
white balance, frames piped into ffmpeg (libx264).  Why: UVC controls set in one process do NOT persist when ffmpeg opens the
device (tested 10/28: mean brightness 99 in-process, 37 again in a fresh ffmpeg), so the recording process has to set them.

  python capture_cv.py --index 2 --out takes.mkv --stop-file STOP [--exposure -5 --gain 24 --brightness 16 --secs 0]
  python capture_cv.py --index 2 --still still.jpg      (one locked-exposure still after a warm-up, for the framing check)

Exposure -5 = 1/32 s (the longest that still allows 30 fps).  Auto white balance is switched off (holds the last value).
Writes <out>.log with the controls it read back and the frame/drop counts.  Stops when --stop-file exists or --secs elapsed.
"""
import argparse, os, subprocess, sys, time
import cv2

W, H, FPS = 1920, 1080, 30
ENC = ["-c:v", "libx264", "-preset", "veryfast", "-crf", "24", "-maxrate", "8M", "-bufsize", "16M", "-pix_fmt", "yuv420p", "-threads", "2"]


def open_locked(index, exposure, gain, brightness, auto=False):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"cannot open DSHOW index {index}")
    cap.set(cv2.CAP_PROP_FPS, FPS)                       # ORDER MATTERS (tested 10/28): fps, size, fourcc = MJPG 1080p29.7; any other order = YUY2 at 5 fps
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    if auto:                                              # wrist cam: leave the camera's own auto exposure alone
        for _ in range(20):
            cap.read()
        return cap, {"AUTO": 1}
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)            # DirectShow: 0.25 = manual
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    cap.set(cv2.CAP_PROP_GAIN, gain)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    for _ in range(20):                                   # let the settings take effect
        cap.read()
    got = {n: cap.get(getattr(cv2, "CAP_PROP_" + n)) for n in ("EXPOSURE", "GAIN", "BRIGHTNESS", "AUTO_WB", "AUTO_EXPOSURE")}
    return cap, got


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--index", type=int, required=True)
    a.add_argument("--out"); a.add_argument("--still"); a.add_argument("--stop-file"); a.add_argument("--secs", type=float, default=0)
    a.add_argument("--exposure", type=float, default=-5); a.add_argument("--gain", type=float, default=24); a.add_argument("--brightness", type=float, default=16)
    a.add_argument("--wb-r", type=float, default=1.0); a.add_argument("--wb-g", type=float, default=1.0); a.add_argument("--wb-b", type=float, default=1.0)   # fixed WB gains: DSHOW ignores WB_TEMPERATURE on the Innomaker (10/30)
    a.add_argument("--auto", action="store_true", help="no manual exposure lock (wrist cam)")
    a.add_argument("--preview", default="", help="window title: show a live 960x540 view while recording")
    a.add_argument("--preview-x", type=int, default=0)
    a = a.parse_args()
    cap, got = open_locked(a.index, a.exposure, a.gain, a.brightness, a.auto)
    if a.preview:
        cv2.namedWindow(a.preview, cv2.WINDOW_NORMAL); cv2.resizeWindow(a.preview, 960, 540); cv2.moveWindow(a.preview, a.preview_x, 40)
    wbf = ["-vf", f"colorchannelmixer=rr={a.wb_r:.3f}:gg={a.wb_g:.3f}:bb={a.wb_b:.3f}"] if (a.wb_r, a.wb_g, a.wb_b) != (1.0, 1.0, 1.0) else []
    if a.still:
        ok, f = cap.read()
        if wbf:
            import numpy as np
            f = np.clip(f * np.array([a.wb_b, a.wb_g, a.wb_r]), 0, 255).astype("uint8")
        cv2.imwrite(a.still, f, [cv2.IMWRITE_JPEG_QUALITY, 92])
        print("still", a.still, "mean", round(float(f.mean()), 1), got)
        cap.release(); return
    log = open(a.out + ".log", "w")
    log.write(f"controls read back: {got}\n"); log.flush()
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "warning", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}", "-framerate", str(FPS),
                           "-use_wallclock_as_timestamps", "1", "-i", "-", *wbf, *ENC, a.out], stdin=subprocess.PIPE, stderr=log)
    open(a.out + ".ready", "w").write("ready")            # capture.py start waits for this before it says "recording"
    t0 = time.time(); n = 0; slow = 0; last = t0
    while True:
        ok, f = cap.read()
        now = time.time()
        if not ok:
            log.write(f"read failed at {now - t0:.1f}s\n"); break
        if now - last > 0.1:
            slow += 1                                     # a gap > 3 frames = a drop
        last = now
        try:
            ff.stdin.write(f.tobytes())
        except OSError:
            log.write("ffmpeg pipe closed\n"); break
        n += 1
        if a.preview and n % 2 == 0:                      # 15 fps view; the recording keeps every frame
            cv2.imshow(a.preview, cv2.resize(f, (960, 540), interpolation=cv2.INTER_AREA))
            cv2.waitKey(1)
        if n % 30 == 0:
            if (a.stop_file and os.path.exists(a.stop_file)) or (a.secs and now - t0 >= a.secs):
                break
    try:
        ff.stdin.close()
    except OSError:
        pass
    ff.wait(timeout=60)
    cap.release()
    log.write(f"frames {n} in {time.time() - t0:.1f}s ({n / max(time.time() - t0, 1e-6):.1f} fps), gaps>100ms {slow}\n"); log.close()


if __name__ == "__main__":
    main()

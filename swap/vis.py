# SPDX-License-Identifier: GPL-3.0-or-later
"""Wrist-cam alignment helpers (10/29). The wrist cam rides on the master and turns with wrist_roll.

Roll-centre trick: rolling the master about its own axis rotates the wrist image about the image point where the master
axis meets the tool plate (points on the axis do not move). So the centre of the image rotation between two roll angles
= where the master will arrive; the tool's hub must sit there. No intrinsics, no hand-eye, no reference image needed.

    python vis.py grab NAME               save wrist (and dock) frames to logs/vis/NAME_*.jpg
    python vis.py rotc A.jpg B.jpg        rotation centre + angle between two wrist frames (ORB + similarity RANSAC)
    python vis.py shift A.jpg B.jpg       pure image translation of the tool region between two frames
"""
import os, subprocess, sys
import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "logs", "vis")
os.makedirs(OUT, exist_ok=True)


def grab(name):
    sys.path.insert(0, HERE)
    import capture as C
    cams = C.find_cameras(); paths = {}
    for n, alt in cams.items():
        p = os.path.join(OUT, f"{name}_{n}.jpg")
        subprocess.run([C.FF, "-y", "-loglevel", "error", *C.cam_args(alt), "-frames:v", "12", "-update", "1", p])
        paths[n] = p
    return paths


def _feat(img, mask=None):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    g = cv2.createCLAHE(2.0, (8, 8)).apply(g)
    orb = cv2.ORB_create(4000, fastThreshold=10)
    return orb.detectAndCompute(g, mask)


def match_sim(a, b, mask=None):
    """Similarity transform b ~ s R a + t from ORB matches, RANSAC. Returns (M 2x3, inlier count, points a, points b)."""
    ka, da = _feat(a, mask); kb, db = _feat(b, None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    m = sorted(bf.match(da, db), key=lambda x: x.distance)[:800]
    pa = np.float32([ka[x.queryIdx].pt for x in m]); pb = np.float32([kb[x.trainIdx].pt for x in m])
    M, inl = cv2.estimateAffinePartial2D(pa, pb, method=cv2.RANSAC, ransacReprojThreshold=4.0, maxIters=5000)
    inl = inl.ravel().astype(bool)
    return M, int(inl.sum()), pa[inl], pb[inl]


def rot_centre(M):
    A = M[:, :2]; t = M[:, 2]
    c = np.linalg.solve(np.eye(2) - A, t)
    ang = np.degrees(np.arctan2(A[1, 0], A[0, 0])); s = np.hypot(A[0, 0], A[1, 0])
    return c, ang, s


def tool_mask(img):
    """Features only from the dark tool plate region (the plate is the big dark blob; background is bright)."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = g.shape
    m = np.zeros_like(g); m[: int(h * 0.75), :] = 255          # the rubber-banded cam's own housing sits in the bottom
    return m


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[0] == "grab":
        print(grab(a[1]))
    elif a[0] in ("rotc", "shift"):
        A, B = cv2.imread(a[1]), cv2.imread(a[2])
        M, n, pa, pb = match_sim(A, B, tool_mask(A))
        if a[0] == "rotc":
            c, ang, s = rot_centre(M)
            print(f"inliers {n}  rotation {ang:+.2f} deg  scale {s:.4f}  centre px ({c[0]:.0f}, {c[1]:.0f})  image {A.shape[1]}x{A.shape[0]}")
        else:
            print(f"inliers {n}  shift px ({M[0,2]:+.1f}, {M[1,2]:+.1f})  rot {np.degrees(np.arctan2(M[1,0], M[0,0])):+.2f}")

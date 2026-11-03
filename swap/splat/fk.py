# SPDX-License-Identifier: GPL-3.0-or-later
"""SO-101 forward kinematics, joint-angle conversions and pinhole camera maths.

Pure numpy; importable under numpy 1.26 and 2.x (only core dtypes, none of the aliases
removed in numpy 2), no OpenCV, no server imports, no hardware access.  Every convention here mirrors
``dashboard/so101_twin.js`` (JOINTS / JOINT_CONFIG) and ``dashboard/so101.urdf`` so the
server-side map and the browser twin agree bit-for-bit (see DESIGN.md §2, RESEARCH.md §5).

Frames
------
* ``base_link``  — URDF root, metres, Z-up.
* ``gripper_link`` — output of the 5-joint chain; the jaws extend along its −Z axis.
* ``gripper_frame_link`` — jaw-tip frame (``gripper_link · Trans(TIP_G) · Ry(π)``).
* camera — OpenCV pinhole (x right, y down, z optical) of the server-rotated 640×480 frame.
* scene — Three.js, millimetres, Y-up: ``(1000·x, 1000·z, −1000·y)``.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Joint tables
# ---------------------------------------------------------------------------

ARM_JOINTS: List[str] = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ALL_JOINTS: List[str] = ARM_JOINTS + ["gripper"]

#: (name, parent, child, xyz [m], rpy [rad]) — literals verbatim from so101_twin.js JOINTS.
#: The truncated ``3.14159`` / ``1.5708`` values are deliberate: they give parity with the twin.
JOINTS: List[Tuple[str, str, str, List[float], List[float]]] = [
    ("shoulder_pan",  "base_link",      "shoulder_link",   [0.0388353, -8.97657e-09, 0.0624],  [3.14159, 4.18253e-17, -3.14159]),
    ("shoulder_lift", "shoulder_link",  "upper_arm_link",  [-0.0303992, -0.0182778, -0.0542],  [-1.5708, -1.5708, 0]),
    ("elbow_flex",    "upper_arm_link", "lower_arm_link",  [-0.11257, -0.028, 1.73763e-16],    [-3.63608e-16, 8.74301e-16, 1.5708]),
    ("wrist_flex",    "lower_arm_link", "wrist_link",      [-0.1349, 0.0052, 3.62355e-17],     [4.02456e-15, 8.67362e-16, -1.5708]),
    ("wrist_roll",    "wrist_link",     "gripper_link",    [5.55112e-17, -0.0611, 0.0181],     [1.5708, 0.0486795, 3.14159]),
    ("gripper",       "gripper_link",   "moving_jaw_link", [0.0202, 0.0188, -0.0234],          [1.5708, -5.24284e-08, -1.41553e-15]),
]

#: Sign applied between encoder/lerobot angle and URDF joint angle (twin JOINT_CONFIG.direction).
DIRECTION: Dict[str, int] = {
    "shoulder_pan": 1, "shoulder_lift": 1, "elbow_flex": 1, "wrist_flex": 1, "wrist_roll": 1, "gripper": 1,
}
# 2026-09-11: shoulder_pan was -1 (a guess from the URDF axis).  Real images say +1: with -1 every
# adjacent-pan pair's optical flow ran opposite to FK (flow-sign votes), and the mat-grid hand-eye fit
# only closed (0.33 deg rms) with the pan mirrored.  so101_twin.js JOINT_CONFIG carries the same flip.

#: Leader-arm encoder midpoints (dashboard ``/servo`` feed; leader_arm.json range midpoints).
LEADER_MID: Dict[str, float] = {
    "shoulder_pan": 2253.5, "shoulder_lift": 1976.5, "elbow_flex": 2081.5, "wrist_flex": 1884.0, "wrist_roll": 2047.5,
}
LEADER_GRIP: Tuple[int, int] = (1568, 2718)

#: Follower-arm encoder midpoints (follower_arm.json range midpoints) and gripper tick range.
FOLLOWER_MID: Dict[str, float] = {
    "shoulder_pan": 2034.0, "shoulder_lift": 2018.0, "elbow_flex": 2010.5, "wrist_flex": 1722.5, "wrist_roll": 2047.5,
}
FOLLOWER_GRIP: Tuple[int, int] = (1270, 2667)

#: Joint zero offsets (URDF radians added after the tick/degree conversion): the calibrated
#: range midpoints are NOT the URDF zero pose.  Fitted 2026-09-11 from a 30-view calibration sweep
#: (mat-grid plane poses + jaw-tip pixel, 0.57 deg / 1.2 mm rms over 8 arm postures incl. wrist roll).
#: so101_twin.js JOINT_CONFIG.offset carries the same values.  Applied by ticks_to_q, lerobot_obs_to_q
#: and their inverses; pass apply_offsets=False for the raw (pre-fit) convention.
ZERO_OFFSET_RAD: Dict[str, float] = {
    "shoulder_lift": -0.112679,   # -6.46 deg
    "elbow_flex": 0.109532,   # +6.28 deg
    "wrist_flex": -0.487227,   # -27.92 deg
    "wrist_roll": 0.119796,   # +6.86 deg
}

#: Moving-jaw angle at 100 % gripper opening (twin updateJoints()).
GRIPPER_MAX_RAD: float = 1.75

TICKS_PER_REV: int = 4096          # STS3215 encoder resolution (continuous wrist_roll wraps mod this)
TICK_MAX: float = 4095.0           # lerobot max_res: deg = (tick − mid) · 360 / 4095
RAD_PER_TICK: float = 2.0 * math.pi / TICK_MAX


# ---------------------------------------------------------------------------
# Rotation / transform helpers
# ---------------------------------------------------------------------------

def Rx(a: float) -> np.ndarray:
    """Rotation about X by ``a`` radians (3×3)."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def Ry(a: float) -> np.ndarray:
    """Rotation about Y by ``a`` radians (3×3)."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def Rz(a: float) -> np.ndarray:
    """Rotation about Z by ``a`` radians (3×3)."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    """URDF fixed-axis rpy (== Three.js Euler 'ZYX'.set(r, p, y)): ``R = Rz(y) · Ry(p) · Rx(r)``."""
    return Rz(y) @ Ry(p) @ Rx(r)


def T_from(R: np.ndarray, t: Sequence[float] | np.ndarray) -> np.ndarray:
    """Homogeneous 4×4 from a 3×3 rotation and a 3-vector translation."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


# ---------------------------------------------------------------------------
# Fixed geometry
# ---------------------------------------------------------------------------

#: Jaw tip in the gripper_link frame (URDF gripper_frame_joint origin), metres.
TIP_G: np.ndarray = np.array([-0.0079, -0.000218121, -0.0981274], dtype=np.float64)

#: gripper_link → gripper_frame_link (fixed joint; +Z of the frame points out along the jaws).
T_GRIPPER_FRAME: np.ndarray = T_from(rpy_to_R(0.0, 3.14159, 0.0), TIP_G)

#: URDF base (m, Z-up) → Three.js scene (mm, Y-up): ``scene = (1000x, 1000z, −1000y)``.
T_SCENE_FROM_URDF: np.ndarray = np.array(
    [[1000.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1000.0, 0.0], [0.0, -1000.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
    dtype=np.float64,
)


# ---------------------------------------------------------------------------
# Angle conversions
# ---------------------------------------------------------------------------

def tick_to_rad(tick: float, mid: float) -> float:
    """Encoder tick → radians, identical to the twin's ``tickToRad``: ``(tick−mid)·360/4095·π/180``."""
    return (float(tick) - float(mid)) * 360.0 / TICK_MAX * (math.pi / 180.0)


def _gripper_pct(tick: float, grip: Tuple[float, float]) -> float:
    """Gripper tick → opening fraction in [0, 1] (clamped)."""
    lo, hi = float(grip[0]), float(grip[1])
    return min(1.0, max(0.0, (float(tick) - lo) / (hi - lo)))


def ticks_to_q(
    ticks: Mapping[str, float],
    mids: Mapping[str, float] = LEADER_MID,
    grip: Tuple[float, float] = LEADER_GRIP,
    apply_offsets: bool = True,
) -> Dict[str, float]:
    """Encoder ticks → URDF joint radians.  Joints absent from ``ticks`` are omitted.

    Defaults are the leader tables (dashboard ``/servo`` feed and the sweep's converted
    reports); pass ``FOLLOWER_MID``/``FOLLOWER_GRIP`` for raw follower ticks.
    """
    q: Dict[str, float] = {}
    for name in ARM_JOINTS:
        if name in ticks:
            q[name] = DIRECTION[name] * tick_to_rad(ticks[name], mids[name]) + (ZERO_OFFSET_RAD.get(name, 0.0) if apply_offsets else 0.0)
    if "gripper" in ticks:
        q["gripper"] = DIRECTION["gripper"] * _gripper_pct(ticks["gripper"], grip) * GRIPPER_MAX_RAD
    return q


def lerobot_obs_to_q(obs: Mapping[str, float], apply_offsets: bool = True) -> Dict[str, float]:
    """lerobot 0.4.4 ``get_observation()`` (``use_degrees=True``) → URDF radians.

    Keys ``'<joint>.pos'`` are degrees about the follower's own mid, ``'gripper.pos'`` is
    0..100.  Missing keys are omitted.
    """
    q: Dict[str, float] = {}
    for name in ARM_JOINTS:
        key = name + ".pos"
        if key in obs:
            q[name] = DIRECTION[name] * math.radians(float(obs[key])) + (ZERO_OFFSET_RAD.get(name, 0.0) if apply_offsets else 0.0)
    if "gripper.pos" in obs:
        pct = min(100.0, max(0.0, float(obs["gripper.pos"])))
        q["gripper"] = DIRECTION["gripper"] * (pct / 100.0) * GRIPPER_MAX_RAD
    return q


def q_to_lerobot_action(q: Mapping[str, float], apply_offsets: bool = True) -> Dict[str, float]:
    """Inverse of :func:`lerobot_obs_to_q`: URDF radians → lerobot action dict (degrees / 0..100)."""
    action: Dict[str, float] = {}
    for name in ARM_JOINTS:
        if name in q:
            action[name + ".pos"] = math.degrees(DIRECTION[name] * (float(q[name]) - (ZERO_OFFSET_RAD.get(name, 0.0) if apply_offsets else 0.0)))
    if "gripper" in q:
        action["gripper.pos"] = DIRECTION["gripper"] * float(q["gripper"]) / GRIPPER_MAX_RAD * 100.0
    return action


def follower_deg_to_leader_ticks(deg: Mapping[str, float], gripper_pct: float) -> Dict[str, int]:
    """Follower lerobot degrees + gripper % → leader-equivalent ticks (6 int keys).

    This is what the sweep reports so the twin and the mapper share one tick convention:
    ``tick_j = round(LEADER_MID_j + deg_j · 4095/360)``, ``gripper = round(1568 + pct/100 · 1150)``.
    All five arm joints must be present in ``deg``.
    """
    ticks: Dict[str, int] = {}
    for name in ARM_JOINTS:
        if name not in deg:
            raise KeyError(f"follower_deg_to_leader_ticks: missing joint '{name}'")
        ticks[name] = int(round(LEADER_MID[name] + float(deg[name]) * TICK_MAX / 360.0))
    lo, hi = LEADER_GRIP
    pct = min(100.0, max(0.0, float(gripper_pct)))
    ticks["gripper"] = int(round(lo + pct / 100.0 * (hi - lo)))
    return ticks


def leader_ticks_to_follower_deg(ticks: Mapping[str, float]) -> Tuple[Dict[str, float], float]:
    """Inverse of :func:`follower_deg_to_leader_ticks`.

    Returns ``(deg, gripper_pct)`` where ``deg`` holds the five arm joints in lerobot degrees and
    ``gripper_pct`` is 0..100 (0.0 when the gripper tick is absent).  All five arm joints must be
    present in ``ticks``.
    """
    deg: Dict[str, float] = {}
    for name in ARM_JOINTS:
        if name not in ticks:
            raise KeyError(f"leader_ticks_to_follower_deg: missing joint '{name}'")
        deg[name] = (float(ticks[name]) - LEADER_MID[name]) * 360.0 / TICK_MAX
    pct = _gripper_pct(ticks["gripper"], LEADER_GRIP) * 100.0 if "gripper" in ticks else 0.0
    return deg, pct


def unwrap_roll(a: float, ref: float) -> float:
    """Shift a continuous wrist_roll tick value by ``k·4096`` so it lies nearest ``ref``.

    ``unwrap_roll(10, 4090) == 4106`` — used for shortest-arc interpolation across the 4095→0 wrap.
    """
    a = float(a)
    k = round((float(ref) - a) / TICKS_PER_REV)
    return a + k * TICKS_PER_REV


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def _joint_T(xyz: Sequence[float], rpy: Sequence[float], theta: float) -> np.ndarray:
    """One twin joint: ``jointOriginGroup(pos=xyz, rot=rpy) → jointPivot(rotation.z=theta)``."""
    return T_from(rpy_to_R(*rpy), xyz) @ T_from(Rz(theta), np.zeros(3))


def fk_all(q: Mapping[str, float]) -> Dict[str, np.ndarray]:
    """Forward kinematics for every link.

    ``q`` maps joint name → URDF radians; missing joints are taken as 0.  Returns
    ``{link: T_base_link}`` (4×4, metres) for ``base_link, shoulder_link, upper_arm_link,
    lower_arm_link, wrist_link, gripper_link, moving_jaw_link, gripper_frame_link``.
    """
    T: Dict[str, np.ndarray] = {"base_link": np.eye(4, dtype=np.float64)}
    for name, parent, child, xyz, rpy in JOINTS:
        T[child] = T[parent] @ _joint_T(xyz, rpy, float(q.get(name, 0.0)))
    T["gripper_frame_link"] = T["gripper_link"] @ T_GRIPPER_FRAME
    return T


def fk_gripper(q: Mapping[str, float]) -> np.ndarray:
    """``T_base_gripper(q)`` (4×4, metres)."""
    return fk_all(q)["gripper_link"]


def camera_pose(q: Mapping[str, float], T_gripper_cam: np.ndarray) -> np.ndarray:
    """``T_base_cam = T_base_gripper(q) · T_gripper_cam`` (hand-eye composition)."""
    return fk_gripper(q) @ np.asarray(T_gripper_cam, dtype=np.float64)


def jaw_tip_base(T_base_gripper: np.ndarray) -> np.ndarray:
    """Jaw-tip position in base_link (3,) for a given ``T_base_gripper``."""
    T = np.asarray(T_base_gripper, dtype=np.float64)
    return T[:3, :3] @ TIP_G + T[:3, 3]


def urdf_m_to_scene_mm(P: np.ndarray | Iterable[float]) -> np.ndarray:
    """URDF base metres → Three.js scene millimetres: ``(x, y, z) → (1000x, 1000z, −1000y)``.

    Accepts a single point or an ``(N, 3)`` array; always returns ``(N, 3)``.
    """
    P2 = np.atleast_2d(np.asarray(P, dtype=np.float64))
    return np.stack([P2[:, 0], P2[:, 2], -P2[:, 1]], axis=1) * 1000.0


# ---------------------------------------------------------------------------
# Pinhole camera (OpenCV convention), no cv2 required
# ---------------------------------------------------------------------------

def K_from_hfov(w: int = 640, h: int = 480, hfov_deg: float = 60.0) -> np.ndarray:
    """Intrinsic matrix for a ``w×h`` image with horizontal FOV ``hfov_deg``.

    ``f = (w/2)/tan(hfov/2)``, principal point at the pixel-centre ``((w−1)/2, (h−1)/2)``.
    """
    f = (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return np.array([[f, 0.0, (w - 1) / 2.0], [0.0, f, (h - 1) / 2.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _dist5(dist: Sequence[float] | np.ndarray | None) -> Tuple[float, float, float, float, float]:
    """Normalise a distortion vector to ``(k1, k2, p1, p2, k3)``; missing tail entries are 0."""
    values = [float(v) for v in np.asarray([] if dist is None else dist, dtype=np.float64).ravel()]
    values = (values + [0.0] * 5)[:5]
    return values[0], values[1], values[2], values[3], values[4]


def distort(xn: np.ndarray, yn: np.ndarray, dist: Sequence[float] | np.ndarray | None) -> Tuple[np.ndarray, np.ndarray]:
    """Apply the OpenCV radial/tangential model ``(k1, k2, p1, p2, k3)`` to normalised coordinates."""
    k1, k2, p1, p2, k3 = _dist5(dist)
    xn = np.asarray(xn, dtype=np.float64)
    yn = np.asarray(yn, dtype=np.float64)
    r2 = xn * xn + yn * yn
    rad = 1.0 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    xd = xn * rad + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    yd = yn * rad + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
    return xd, yd


def undistort_normalized(
    K: np.ndarray, dist: Sequence[float] | np.ndarray | None, uv: np.ndarray, iters: int = 10
) -> np.ndarray:
    """Pixels → undistorted normalised coordinates ``(x/z, y/z)``, ``(N, 2)``.

    Same fixed-point scheme as ``cv2.undistortPoints``.
    """
    K = np.asarray(K, dtype=np.float64)
    uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
    xd = (uv[:, 0] - K[0, 2]) / K[0, 0]
    yd = (uv[:, 1] - K[1, 2]) / K[1, 1]
    xn, yn = xd.copy(), yd.copy()
    for _ in range(int(iters)):
        xdd, ydd = distort(xn, yn, dist)
        xn -= xdd - xd
        yn -= ydd - yd
    return np.stack([xn, yn], axis=1)


def backproject_rays(K: np.ndarray, dist: Sequence[float] | np.ndarray | None, uv: np.ndarray) -> np.ndarray:
    """Pixels → unit rays in the camera frame, ``(N, 3)``."""
    n = undistort_normalized(K, dist, uv)
    d = np.concatenate([n, np.ones((n.shape[0], 1), dtype=np.float64)], axis=1)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def project(
    K: np.ndarray, dist: Sequence[float] | np.ndarray | None, P_cam: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Camera-frame points ``(N, 3)`` → ``(uv (N, 2), z (N,))``.

    Equivalent to ``cv2.projectPoints(P, rvec=0, tvec=0, K, dist)``.  Points with ``z == 0``
    yield non-finite pixels (numpy division warnings suppressed); callers gate on ``z``.
    """
    K = np.asarray(K, dtype=np.float64)
    P = np.atleast_2d(np.asarray(P_cam, dtype=np.float64))
    z = P[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        xd, yd = distort(P[:, 0] / z, P[:, 1] / z, dist)
    uv = np.stack([K[0, 0] * xd + K[0, 2], K[1, 1] * yd + K[1, 2]], axis=1)
    return uv, z

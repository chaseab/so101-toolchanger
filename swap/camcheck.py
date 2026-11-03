# SPDX-License-Identifier: GPL-3.0-or-later
"""Camera sanity (11/02 eve, after the OBS-placeholder incident: take 203227 recorded 'OBS Virtual Camera' and the dock
check compared placeholder frames, reporting 'docked 1.0').

    dshow_names()            DirectShow device names in index order (pygrabber; same order OpenCV CAP_DSHOW uses)
    is_virtual(name)         OBS / EOS / any '*Virtual*' device: never a dock camera
    real_indices()           indices of non-virtual devices (None if names can't be listed)
    frame_problem(a, b)      None if two consecutive live frames look like a real camera, else a reason string
"""
import io, os

HERE = os.path.dirname(os.path.abspath(__file__))
BAD = ("OBS", "EOS", "VIRTUAL")


def dshow_names():
    try:
        from pygrabber.dshow_graph import FilterGraph
        return list(FilterGraph().get_input_devices())
    except Exception:
        return None


def is_virtual(name):
    n = str(name).upper()
    return any(b in n for b in BAD)


def real_indices():
    names = dshow_names()
    return None if names is None else [i for i, n in enumerate(names) if not is_virtual(n)]


def _grey(b):
    import numpy as np
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(b)).convert("L").resize((160, 90)), float)


def frame_problem(a, b=None):
    """a, b: jpg bytes of two frames >= 0.5 s apart (b optional). Reasons: placeholder (matches the saved OBS image),
    uniform (no texture), frozen (byte-identical frames: a real sensor always has noise)."""
    import numpy as np
    g = _grey(a)
    if g.std() < 6:
        return f"uniform frame (std {g.std():.1f})"
    ref = os.path.join(HERE, "refs", "invalid_obs_placeholder.jpg")
    if os.path.exists(ref):
        r = _grey(open(ref, "rb").read())
        x, y = g - g.mean(), r - r.mean()
        c = float((x * y).sum() / (np.linalg.norm(x) * np.linalg.norm(y) + 1e-9))
        if c > 0.9:
            return f"OBS placeholder frame (corr {c:.2f})"
    if b is not None and a == b:
        return "frozen frame (two frames byte-identical)"
    return None

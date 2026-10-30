# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Gear helpers for JG2 (involute spur, straight miter bevel by scaled loft, rack).  2026-10-30, pm-eng CAD worker.
Printable PLA teeth: module 1.0, 20 deg, tooth thickness cut by BACKLASH_ARC at the pitch line.
Bevel = Tredgold-style loft: the spur outline at the outer (back) end is scaled about the cone apex, so every tooth edge runs to the apex.
"""
from math import pi, cos, sin, tan, acos, radians

import numpy as np
from build123d import Polygon, extrude, Pos, Solid

M, ALPHA, BACKLASH_ARC = 1.0, radians(20.0), 0.2      # mm of tooth thickness removed per gear at the pitch circle (rack too)


def inv(a):
    return tan(a) - a


def spur_outline(z, m=M, n_flank=7, backlash=BACKLASH_ARC):
    """Closed involute outline (list of (x,y)), tooth 0 centred on +x."""
    rp = m * z / 2
    rb = rp * cos(ALPHA)
    ra = rp + m
    rf = rp - 1.25 * m
    d = backlash / (2 * rp)

    def half(r):
        a = acos(min(1.0, rb / r))
        return pi / (2 * z) - d + inv(ALPHA) - inv(a)

    r0 = max(rb, rf)
    rs = np.linspace(r0, ra, n_flank)
    pts = []
    for i in range(z):
        phi = 2 * pi * i / z
        seq = []
        if rf < rb:
            seq.append((rf, phi - half(rb)))
        for r in rs:
            seq.append((r, phi - half(r)))
        for r in rs[::-1]:
            seq.append((r, phi + half(r)))
        if rf < rb:
            seq.append((rf, phi + half(rb)))
        pts += [(r * cos(a), r * sin(a)) for r, a in seq]
        a_end = phi + half(r0)
        a_nxt = phi + 2 * pi / z - half(r0)
        for k in (1, 2):
            a = a_end + (a_nxt - a_end) * k / 3
            pts.append((rf * cos(a), rf * sin(a)))
    return pts


def spur(z, width, m=M):
    """Spur gear, axis +z from 0 to width, tooth 0 centred on +x."""
    return extrude(Polygon(*spur_outline(z, m), align=None), amount=width)


def bevel(z, h_out, h_in, m=M):
    """Miter bevel (pitch cone half-angle 45 deg): axis +z, apex at z=0, outer (back) end at h_out, inner end at h_in (< h_out).
    Outline radii scale with the distance to the apex (pitch radius at h is h).  Tooth 0 centred on +x."""
    rp = m * z / 2
    pts = spur_outline(z, m)
    s_out, s_in = h_out / rp, h_in / rp
    w_out = Pos(0, 0, h_out) * Polygon(*[(x * s_out, y * s_out) for x, y in pts], align=None)
    w_in = Pos(0, 0, h_in) * Polygon(*[(x * s_in, y * s_in) for x, y in pts], align=None)
    return Solid.make_loft([w_out.wire(), w_in.wire()], True)


def rack(centres, width, body_depth, m=M, backlash=BACKLASH_ARC):
    """Rack along local +y (tooth centres y in `centres`, spacing pi*m), pitch line x = 0, tips toward -x at x = -m,
    root at +1.25 m, body to x = 1.25 m + body_depth, extruded along z 0..width."""
    p = pi * m
    th = p / 4 - backlash / 2
    hr = th + 1.25 * m * tan(ALPHA)
    ht = th - 1.0 * m * tan(ALPHA)
    xb = 1.25 * m + body_depth
    ys = list(centres)
    pts = [(xb, ys[0] - p / 2), (1.25 * m, ys[0] - p / 2)]
    for yc in ys:
        pts += [(1.25 * m, yc - hr), (-m, yc - ht), (-m, yc + ht), (1.25 * m, yc + hr)]
    pts += [(1.25 * m, ys[-1] + p / 2), (xb, ys[-1] + p / 2)]
    out = [pts[0]]
    for q in pts[1:]:
        if abs(q[0] - out[-1][0]) > 1e-9 or abs(q[1] - out[-1][1]) > 1e-9:
            out.append(q)
    return extrude(Polygon(*out[::-1], align=None), amount=width)      # CCW so the extrusion goes +z

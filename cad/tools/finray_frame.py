# SPDX-License-Identifier: CERN-OHL-W-2.0
"""2D geometrically nonlinear (corotational Euler-Bernoulli) frame solver for fin ray finger profiles.

Units: mm, N, MPa (N/mm^2). Coordinates: x along the finger from the clamped base (x=0) to the tip (x=L);
y is the height above the back flank; the object sits on the +y side and moves in -y (approach).

API
---
    p = FinRay(L=70, H=16, h_tip=2, t_front=1.2, t_back=1.2, n_ribs=6, t_rib=0.9, rib_angle=10)
    res = simulate(p, Indenter("flat", xc=35, width=25), depths=(5, 8))

rib_angle sign: + means the rib's FRONT-flank end is closer to the TIP than its back-flank end (the rib leans
toward the tip going from back flank to front flank). 0 = ribs perpendicular to the back flank.
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field, asdict
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

E_TPU95A = 26.0  # MPa, default (param)


# ----------------------------------------------------------------------------- geometry
@dataclass
class FinRay:
    L: float = 70.0          # length along the back flank (base clamp to tip)
    H: float = 16.0          # base height (back flank to front flank at x=0)
    h_tip: float = 2.0       # height at the tip
    t_front: float = 1.2
    t_back: float = 1.2
    n_ribs: int = 6
    t_rib: float = 0.9
    rib_angle: float = 0.0   # deg, + leans toward tip (see module doc)
    b: float = 16.0          # out-of-plane width
    E: float = E_TPU95A
    h_el: float = 1.5        # target element length
    plate: bool = False      # rigid backstop under the back flank at y = -plate_gap (v1: gap 0, resting)
    plate_gap: float = 0.0
    x_clamp: float = 0.0     # clamp position (both flanks clamped at x = x_clamp); geometry starts here
    rib_layout: str = "back"  # "back": feet at L*i/(n+1) on the back flank, ribs whose front end falls outside
                              #         (0.5, L-0.5) are DROPPED (legacy; steep negative angles lose ribs)
                              # "fit":  all n ribs kept, feet spaced evenly over the range where both ends lie
                              #         inside [rib_margin, L - rib_margin]
    rib_margin: float = 2.0


def build_mesh(p: FinRay):
    """Return nodes (n,2), elements (m,2), thickness (m,), groups dict, special node ids."""
    L, H, ht = p.L, p.H, p.h_tip
    front = lambda x: H + (ht - H) * x / L
    a = math.radians(p.rib_angle)
    k = (ht - H) / L
    den = math.cos(a) - k * math.sin(a)

    def rib_front(xb):
        # rib line: (xb + s sin a, s cos a); intersect y = front(x):  s (cos a - k sin a) = H + k xb
        s = (H + k * xb) / den
        return xb + s * math.sin(a), s

    ribs = []
    if p.rib_layout == "fit" and p.n_ribs > 0:
        mg = p.rib_margin
        xs = np.linspace(0.0, L, 4001)
        okx = [x for x in xs if mg <= x <= L - mg and mg <= rib_front(x)[0] <= L - mg and rib_front(x)[1] > 0.3]
        if okx:
            lo, hi = min(okx), max(okx)
            n = p.n_ribs
            feet = [0.5 * (lo + hi)] if n == 1 else [lo + (hi - lo) * i / (n - 1) for i in range(n)]
            ribs = [(xb, rib_front(xb)[0]) for xb in feet]
    else:
        for i in range(1, p.n_ribs + 1):
            xb = L * i / (p.n_ribs + 1)
            xf, s = rib_front(xb)
            if 0.5 < xf < L - 0.5 and s > 0.3:
                ribs.append((xb, xf))
    nodes, key = [], {}

    def node(x, y):
        kk = (round(x, 6), round(y, 6))
        if kk not in key:
            key[kk] = len(nodes)
            nodes.append((x, y))
        return key[kk]

    elems, thick, grp = [], [], []

    def polyline(pts, t, g):
        ids = [node(*pts[0])]
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(math.ceil(seg / p.h_el)))
            for j in range(1, n + 1):
                ids.append(node(x0 + (x1 - x0) * j / n, y0 + (y1 - y0) * j / n))
        for i0, i1 in zip(ids[:-1], ids[1:]):
            elems.append((i0, i1)); thick.append(t); grp.append(g)
        return ids

    xs_back = sorted({0.0, L} | {r[0] for r in ribs})
    xs_front = sorted({0.0, L} | {r[1] for r in ribs})
    back_ids = polyline([(x, 0.0) for x in xs_back], p.t_back, "back")
    front_ids = polyline([(x, front(x)) for x in xs_front], p.t_front, "front")
    for xb, xf in ribs:
        polyline([(xb, 0.0), (xf, front(xf))], p.t_rib, "rib")
    tip_front = node(L, ht)
    tip_back = node(L, 0.0)
    if ht > 1e-6:
        polyline([(L, 0.0), (L, ht)], max(p.t_front, p.t_back), "tip")
    X = np.array(nodes, float)
    X[:, 0] += p.x_clamp
    return dict(X=X, conn=np.array(elems, int), t=np.array(thick, float), grp=np.array(grp),
                front=np.array(sorted(set(front_ids), key=lambda i: nodes[i][0])),
                back=np.array(sorted(set(back_ids), key=lambda i: nodes[i][0])),
                base=[node(0.0, 0.0), node(0.0, H)], tip_front=tip_front, tip_back=tip_back, ribs=ribs)


# ----------------------------------------------------------------------------- element kernel
def element_state(X, U, conn, L0, c0, s0, EA, EI, want_K=True):
    Uu = U.reshape(-1, 3)
    i, j = conn[:, 0], conn[:, 1]
    d = (X[j] + Uu[j, :2]) - (X[i] + Uu[i, :2])
    Ln = np.hypot(d[:, 0], d[:, 1])
    c, s = d[:, 0] / Ln, d[:, 1] / Ln
    dbeta = np.arctan2(c0 * s - s0 * c, c0 * c + s0 * s)
    th1 = Uu[i, 2] - dbeta
    th2 = Uu[j, 2] - dbeta
    ul = (Ln ** 2 - L0 ** 2) / (Ln + L0)
    N = EA / L0 * ul
    M1 = EI / L0 * (4 * th1 + 2 * th2)
    M2 = EI / L0 * (2 * th1 + 4 * th2)
    z0 = np.zeros_like(c)
    r = np.stack([-c, -s, z0, c, s, z0], 1)
    zv = np.stack([s, -c, z0, -s, c, z0], 1)
    B1 = zv * (-1.0 / Ln)[:, None]            # d(beta)-related row part: [-s/L, c/L, 0, s/L, -c/L, 0]
    Bm1 = B1.copy(); Bm1[:, 2] += 1.0
    Bm2 = B1.copy(); Bm2[:, 5] += 1.0
    f = r * N[:, None] + Bm1 * M1[:, None] + Bm2 * M2[:, None]
    if not want_K:
        return f, None, (N, M1, M2, Ln)
    k11 = (EA / L0)[:, None, None]
    Km = (k11 * r[:, :, None] * r[:, None, :]
          + (EI / L0 * 4)[:, None, None] * (Bm1[:, :, None] * Bm1[:, None, :] + Bm2[:, :, None] * Bm2[:, None, :])
          + (EI / L0 * 2)[:, None, None] * (Bm1[:, :, None] * Bm2[:, None, :] + Bm2[:, :, None] * Bm1[:, None, :]))
    Kg = ((N / Ln)[:, None, None] * zv[:, :, None] * zv[:, None, :]
          + ((M1 + M2) / Ln ** 2)[:, None, None] * (r[:, :, None] * zv[:, None, :] + zv[:, :, None] * r[:, None, :]))
    return f, Km + Kg, (N, M1, M2, Ln)


# ----------------------------------------------------------------------------- indenters
@dataclass
class Indenter:
    kind: str = "flat"       # "flat" | "cyl" | "none"
    xc: float = 35.0         # centre x of the face / cylinder axis
    width: float = 25.0      # flat face width along x
    r: float = 12.0          # cylinder radius
    rc: float = 1.0          # edge radius of the flat face
    mu: float = 0.0          # Coulomb friction coefficient ("tflat" only; regularised stick, see es)
    # --- "tflat": flat face rotated by phi, approaching along its own normal -------------------------
    phi: float | None = None  # face angle from +x (deg). None -> parallel to the undeformed front flank
    s_off: float = 0.0        # face centre offset along the flank from the front-flank mid-length point (mm, + toward tip)
    es: float = 0.05          # friction regularisation slip (mm): Ft = -mu*Fn*tanh(slip/es)


class Model:
    def __init__(self, p: FinRay | None = None, mesh=None, kp=None):
        self.p = p
        self.m = mesh if mesh is not None else build_mesh(p)
        X, conn, t = self.m["X"], self.m["conn"], self.m["t"]
        E = p.E if p else E_TPU95A
        b = p.b if p else 16.0
        self.X, self.conn, self.t, self.E, self.b = X, conn, t, E, b
        d = X[conn[:, 1]] - X[conn[:, 0]]
        self.L0 = np.hypot(d[:, 0], d[:, 1])
        self.c0, self.s0 = d[:, 0] / self.L0, d[:, 1] / self.L0
        self.EA = E * b * t
        self.EI = E * b * t ** 3 / 12
        n = len(X)
        self.ndof = 3 * n
        dofs = np.stack([3 * conn[:, 0], 3 * conn[:, 0] + 1, 3 * conn[:, 0] + 2,
                         3 * conn[:, 1], 3 * conn[:, 1] + 1, 3 * conn[:, 1] + 2], 1)
        self.edofs = dofs
        self.rows = np.repeat(dofs, 6, axis=1).ravel()
        self.cols = np.tile(dofs, (1, 6)).ravel()
        self.fixed = np.zeros(self.ndof, bool)
        self.kp = kp if kp is not None else 500.0
        self.eps_c = 0.01
        self.fext = np.zeros(self.ndof)
        self.anchor = {}      # friction stick anchors (node -> point), updated after converged steps
        self.P_conv = None    # node positions at the last converged step

    def fix_node(self, i, dofs=(0, 1, 2)):
        for d in dofs:
            self.fixed[3 * i + d] = True

    # smoothed penalty law: force phi(g) for penetration g (C1 at g=0), and its derivative
    def _law(self, g):
        e = self.eps_c
        if g <= 0:
            return 0.0, 0.0
        if g < e:
            return self.kp * g * g / (2 * e), self.kp * g / e
        return self.kp * (g - e / 2), self.kp

    def _point_vs_circle(self, k_node, P, cen, r, fc, rows, cols, vals, outside=True):
        """Node P against circle (outside=True: node must stay outside the disc)."""
        dv = P - cen
        dist = math.hypot(dv[0], dv[1])
        g = r - dist
        phi, dphi = self._law(g)
        if phi <= 0:
            return 0.0
        nrm = dv / dist
        F = phi * nrm
        dd = [3 * k_node, 3 * k_node + 1]
        fc[dd] += F
        Kc = dphi * np.outer(nrm, nrm) - phi / dist * (np.eye(2) - np.outer(nrm, nrm))
        for a_ in range(2):
            for b_ in range(2):
                rows.append(dd[a_]); cols.append(dd[b_]); vals.append(Kc[a_, b_])
        return -F[1]

    # ---- tilted flat face ("tflat") -------------------------------------------------------------
    def tflat_frame(self, ind):
        """(c_ref, e, a): reference point on the undeformed front flank, face tangent, approach direction."""
        P = self.X[self.m["front"]]
        p0, p1 = P[0], P[-1]                       # base end and tip end of the front flank
        u = (p1 - p0) / np.linalg.norm(p1 - p0)
        phi = math.atan2(u[1], u[0]) if ind.phi is None else math.radians(ind.phi)
        e = np.array([math.cos(phi), math.sin(phi)])
        a = np.array([math.sin(phi), -math.cos(phi)])  # e rotated -90 deg: points into the finger
        c_ref = 0.5 * (p0 + p1) + ind.s_off * u
        return c_ref, e, a

    def tflat_start(self, ind):
        """Offset t0 so the face at c_ref + t0*a just touches the undeformed front flank."""
        c_ref, e, a = self.tflat_frame(ind)
        P = self.X[self.m["front"]]
        xi, eta = (P - c_ref) @ e, (P - c_ref) @ a
        hw, rc = ind.width / 2, ind.rc
        best = -1e9
        for x, h in zip(xi, eta):
            if abs(x) <= hw - rc:
                best = max(best, -h)
            elif abs(x) < hw:
                dx = abs(x) - (hw - rc)
                best = max(best, -h + math.sqrt(rc * rc - dx * dx) - rc)
        return -best

    def _tflat(self, U, ind, delta, t0, fc, rows, cols, vals, info):
        c_ref, e, a = self.tflat_frame(ind)
        c = c_ref + (t0 + delta) * a
        ids = self.m["front"]
        P = self.X[ids] + U.reshape(-1, 3)[ids, :2]
        hw, rc = ind.width / 2, ind.rc
        fn = ft = 0.0
        for k in range(len(ids)):
            d = P[k] - c
            xi, eta = d @ e, d @ a
            dd = [3 * ids[k], 3 * ids[k] + 1]
            if abs(xi) <= hw - rc:
                phi_, dphi = self._law(-eta)
                if phi_ <= 0:
                    continue
                nrm = a
                F = phi_ * a
                Kc = dphi * np.outer(a, a)
            elif abs(xi) < hw + rc:
                cen = c + (1.0 if xi > 0 else -1.0) * (hw - rc) * e - rc * a
                dv = P[k] - cen
                if dv @ a < 0:          # beside the block below the corner centre -> not modelled
                    continue
                dist = math.hypot(dv[0], dv[1])
                phi_, dphi = self._law(rc - dist)
                if phi_ <= 0:
                    continue
                nrm = dv / dist
                F = phi_ * nrm
                Kc = dphi * np.outer(nrm, nrm) - phi_ / dist * (np.eye(2) - np.outer(nrm, nrm))
            else:
                continue
            if ind.mu > 0:
                anc = self.anchor.get(int(ids[k]))
                if anc is None:
                    anc = self.P_conv[ids[k]] if self.P_conv is not None else P[k]
                T = math.tanh(((P[k] - anc) @ e) / ind.es)
                Ft = -ind.mu * phi_ * T * e
                F = F + Ft
                # K = -dF/dp ; d(phi)/dp = -dphi*nrm ; d(T)/dp = (1-T^2)/es * e
                Kc = Kc + ind.mu * (-T * dphi * np.outer(e, nrm) + phi_ * (1 - T * T) / ind.es * np.outer(e, e))
                ft += float(Ft @ e)
            fc[dd] += F
            for i_ in range(2):
                for j_ in range(2):
                    rows.append(dd[i_]); cols.append(dd[j_]); vals.append(Kc[i_, j_])
            info["active"].append(ids[k]); fn += float(F @ a)
        info["force"] = fn
        info["force_t"] = ft

    def contact(self, U, ind, delta, y0):
        """Returns (force ON the structure, tangent triplets, info). info['force'] = total push on the indenter (+)."""
        fc = np.zeros(self.ndof)
        rows, cols, vals = [], [], []
        Uu = U.reshape(-1, 3)
        info = dict(active=[], force=0.0)
        if ind is not None and ind.kind != "none":
            ids = self.m["front"]
            P = self.X[ids] + Uu[ids, :2]
            ftot = 0.0
            if ind.kind == "flat":
                yf = y0 - delta
                rc = ind.rc
                xa, xb = ind.xc - ind.width / 2 + rc, ind.xc + ind.width / 2 - rc
                for k in range(len(ids)):
                    x, y = P[k]
                    if xa <= x <= xb:
                        phi, dphi = self._law(y - yf)
                        if phi > 0:
                            dof = 3 * ids[k] + 1
                            fc[dof] += -phi
                            rows.append(dof); cols.append(dof); vals.append(dphi)
                            info["active"].append(ids[k]); ftot += phi
                        continue
                    cen = np.array([xa if x < xa else xb, yf + rc])
                    if P[k][1] > cen[1]:
                        continue
                    f = self._point_vs_circle(ids[k], P[k], cen, rc, fc, rows, cols, vals)
                    if f:
                        info["active"].append(ids[k]); ftot += f
            elif ind.kind == "tflat":
                self._tflat(U, ind, delta, y0, fc, rows, cols, vals, info)
                ftot = info["force"]
            elif ind.kind == "cyl":
                cen = np.array([ind.xc, y0 - delta])
                for k in range(len(ids)):
                    f = self._point_vs_circle(ids[k], P[k], cen, ind.r, fc, rows, cols, vals)
                    if f:
                        info["active"].append(ids[k]); ftot += f
            info["force"] = float(ftot)
        if self.p is not None and self.p.plate:
            ids = self.m["back"]
            y = self.X[ids, 1] + Uu[ids, 1]
            for k in range(len(ids)):
                phi, dphi = self._law(-y[k] - self.p.plate_gap)
                if phi > 0:
                    dof = 3 * ids[k] + 1
                    fc[dof] += phi
                    rows.append(dof); cols.append(dof); vals.append(dphi)
        return fc, (rows, cols, vals), info

    def residual_tangent(self, U, ind, delta, y0, want_K=True):
        f, Ke, st = element_state(self.X, U, self.conn, self.L0, self.c0, self.s0, self.EA, self.EI, want_K)
        fint = np.zeros(self.ndof)
        np.add.at(fint, self.edofs.ravel(), f.ravel())
        fc, (cr, cc, cv), info = self.contact(U, ind, delta, y0)
        R = fint - self.fext - fc
        K = None
        if want_K:
            K = sp.coo_matrix((np.concatenate([Ke.ravel(), np.asarray(cv, float)]),
                               (np.concatenate([self.rows, np.asarray(cr, int)]),
                                np.concatenate([self.cols, np.asarray(cc, int)]))),
                              shape=(self.ndof, self.ndof)).tocsr()
        return R, K, st, info

    def newton(self, U, ind, delta, y0, tol=1e-7, maxit=40):
        free = ~self.fixed
        scale = max(1.0, float(np.max(self.EI / self.L0 ** 2)))
        for it in range(maxit):
            R, K, st, info = self.residual_tangent(U, ind, delta, y0)
            r = np.linalg.norm(R[free])
            fref = max(1e-3, abs(info["force"]), np.linalg.norm(self.fext))
            if r < tol * scale + 1e-6 * fref and it > 0:
                return U, True, it, st, info
            Kf = K[free][:, free]
            try:
                dU = spla.spsolve(Kf.tocsc(), -R[free])
            except Exception:
                return U, False, it, st, info
            if not np.all(np.isfinite(dU)):
                return U, False, it, st, info
            # limit rotation increments for robustness
            mx = np.max(np.abs(dU))
            if mx > 2.0:
                dU *= 2.0 / mx
            U = U.copy(); U[free] += dU
        R, K, st, info = self.residual_tangent(U, ind, delta, y0)
        ok = np.linalg.norm(R[free]) < 1e-4 * max(1.0, abs(info["force"]))
        return U, ok, maxit, st, info


def initial_contact_height(X, front_ids, ind: Indenter):
    P = X[front_ids]
    if ind.kind == "flat":
        xa, xb, rc = ind.xc - ind.width / 2 + ind.rc, ind.xc + ind.width / 2 - ind.rc, ind.rc
        best = -1e9
        for x, y in P:
            if xa <= x <= xb:
                best = max(best, y)
            else:
                dx = x - (xa if x < xa else xb)
                if abs(dx) < rc:
                    best = max(best, y + math.sqrt(rc ** 2 - dx ** 2) - rc)
        return float(best)
    if ind.kind == "cyl":
        dx = P[:, 0] - ind.xc
        k = np.abs(dx) < ind.r
        return float(np.max(P[k, 1] + np.sqrt(ind.r ** 2 - dx[k] ** 2)))
    return 0.0


def _metrics(model: Model, U, st, info, ind):
    N, M1, M2, Ln = st
    t = model.t
    eps = np.abs(N) / (model.E * model.b * t) + np.maximum(np.abs(M1), np.abs(M2)) * 6 / (model.E * model.b * t ** 2)
    Uu = U.reshape(-1, 3)
    m = model.m
    act = set(int(a) for a in info["active"])
    # contact length: front elements with both ends in contact
    fr = m["front"]
    xy = model.X[fr] + Uu[fr, :2]
    clen = 0.0
    for a, b_ in zip(fr[:-1], fr[1:]):
        if a in act and b_ in act:
            clen += float(np.hypot(*(model.X[b_] + Uu[b_, :2] - model.X[a] - Uu[a, :2])))
    back = m["back"]
    yb = model.X[back, 1] + Uu[back, 1]
    a_vec = model.tflat_frame(ind)[2] if (ind is not None and ind.kind == "tflat") else np.array([0.0, -1.0])
    tip_toward = float(-(Uu[m["tip_front"], :2] @ a_vec))   # tip displacement toward the object (+ = wraps)
    return dict(force=info["force"], force_t=info.get("force_t", 0.0), tip_toward=tip_toward,
                n_contact=len(act), contact_len=clen,
                tip_dy=float(Uu[m["tip_front"], 1]), tip_dx=float(Uu[m["tip_front"], 0]),
                back_min_y=float(yb.min()), back_max_y=float(yb.max()),
                eps_max=float(eps.max()), eps_grp={g: float(eps[m["grp"] == g].max()) for g in np.unique(m["grp"])})


def simulate(p: FinRay, ind: Indenter, depths=(5.0, 8.0), step=0.25, keep_shapes=False, verbose=False):
    """Push the indenter into the front flank (displacement control). Returns metrics at each depth in `depths`."""
    model = Model(p)
    for i in model.m["base"]:
        model.fix_node(i)
    y0 = model.tflat_start(ind) if ind.kind == "tflat" else initial_contact_height(model.X, model.m["front"], ind)
    U = np.zeros(model.ndof)
    model.P_conv = model.X.copy()
    out = dict(params=asdict(p), indenter=asdict(ind), at={}, curve=[], shapes={}, ok=True)
    dmax = max(depths)
    targets = sorted(set(list(depths)))
    delta, h = 0.0, step
    while delta < dmax - 1e-9:
        nxt = min(delta + h, dmax)
        for t_ in targets:
            if delta < t_ - 1e-9 < nxt:
                nxt = t_
        U1, ok, it, st, info = model.newton(U, ind, nxt, y0)
        if not ok:
            h /= 2
            if h < 1e-3:
                out["ok"] = False
                break
            continue
        U, delta = U1, nxt
        act_ids = set(int(i) for i in info["active"])
        for i in act_ids:
            if i not in model.anchor:
                model.anchor[i] = model.P_conv[i].copy()
        for i in list(model.anchor):
            if i not in act_ids:
                del model.anchor[i]
        model.P_conv = model.X + U.reshape(-1, 3)[:, :2]
        if it < 6:
            h = min(step, h * 1.5)
        met = _metrics(model, U, st, info, ind)
        out["curve"].append((delta, met["force"], met["tip_dy"], met["contact_len"], met["tip_toward"]))
        if any(abs(delta - t_) < 1e-9 for t_ in targets):
            out["at"][round(delta, 3)] = met
            if keep_shapes:
                out["shapes"][round(delta, 3)] = (model.X + U.reshape(-1, 3)[:, :2]).copy()
    out["mesh"] = model.m if keep_shapes else None
    out["y0"] = y0
    return out


# ----------------------------------------------------------------------------- generic helpers for validation
def cantilever_model(Lb=100.0, n=40, E=100.0, b=10.0, t=2.0, angle=0.0):
    ca, sa = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    X = np.array([[Lb * k / n * ca, Lb * k / n * sa] for k in range(n + 1)])
    conn = np.array([(k, k + 1) for k in range(n)])
    mesh = dict(X=X, conn=conn, t=np.full(n, t), grp=np.array(["beam"] * n), front=np.arange(n + 1),
                back=np.arange(n + 1), base=[0], tip_front=n, tip_back=n)
    p = FinRay(E=E, b=b)
    mdl = Model(p, mesh=mesh)
    mdl.p = None  # no plate
    mdl.E, mdl.b = E, b
    return mdl


def solve_force(model: Model, F_dir_dof, F_total, nsteps=20):
    U = np.zeros(model.ndof)
    for k in range(1, nsteps + 1):
        model.fext[:] = 0
        for dof, frac in F_dir_dof:
            model.fext[dof] = F_total * frac * k / nsteps
        U, ok, it, st, info = model.newton(U, None, 0.0, 0.0, tol=1e-10, maxit=60)
        if not ok:
            raise RuntimeError(f"no convergence at step {k}")
    return U

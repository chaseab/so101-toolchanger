# SPDX-License-Identifier: CERN-OHL-W-2.0
"""Solid 2D fin ray profile matching the sim's centreline model (finray_frame.build_mesh, vendored next to this file), plus the TPU root block.

Local frame: x along the back flank from the clamp (x=0) to the tip (x=L); y = height; the front flank faces +y.
Flanks/ribs are the sim's centrelines buffered by t/2 (flat caps), unioned, with concave corners filleted.
The root block (x in [-ROOT_BLOCK, 0]) is solid TPU clamped between the carrier's cheek plates.
"""
import math
import numpy as np
from shapely.geometry import LineString, Polygon, box
from shapely.ops import unary_union

# v3 (arbiter/ continuum FE, 2026-09-25): H24, 0.8 ribs at -55, fillet 0.3 -> curls in at every placement, 8.3 % strain at 5 mm
DESIGN = dict(L=50.0, H=24.0, h_tip=1.5, t_front=1.2, t_back=2.0, n_ribs=6, t_rib=0.8, rib_angle=-55.0,
              width=16.0, fillet=0.3, root_block=6.0, bolt_d=3.4,
              pitch_deg=15.0)   # mount pitch: block face meets the front flank at 15 deg, opening toward the tip (sim tilt study)


def rib_lines(d=DESIGN):
    """Rib centrelines taken from the sim itself (rib_layout='fit' keeps all n ribs), so CAD == simulated."""
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from finray_frame import FinRay, build_mesh
    L, H, ht = d["L"], d["H"], d["h_tip"]
    m = build_mesh(FinRay(L=L, H=H, h_tip=ht, t_front=d["t_front"], t_back=d["t_back"], n_ribs=d["n_ribs"],
                          t_rib=d["t_rib"], rib_angle=d["rib_angle"], rib_layout="fit"))
    k = (ht - H) / L
    return [((xb, 0.0), (xf, H + k * xf)) for xb, xf in m["ribs"]]


def teeth(d=DESIGN):
    """Optional barbs on the outer face of the front flank (finray2 10/31): sawtooth ridges across the pull direction.
    d['teeth'] = (height, pitch, x_from, x_to) or absent/None.  Each tooth rises steeply on its ROOT side and ramps
    down toward the tip: an object pushed in (toward the root) rides up the ramps; one pulled out (toward the tip) meets
    the steep faces."""
    spec = d.get("teeth")
    if not spec:
        return []
    th, pitch, x0, x1 = spec
    L, H, ht = d["L"], d["H"], d["h_tip"]
    ux, uy = L / math.hypot(L, ht - H), (ht - H) / math.hypot(L, ht - H)      # along the front flank, root -> tip
    nx, ny = -uy, ux                                                           # outward normal (+y side)
    off = d["t_front"] / 2 - 0.05                                              # sit on (and overlap) the outer face
    out, s = [], 0.0
    s_end = (x1 - x0) / ux
    while s + pitch <= s_end + 1e-9:
        x, y = x0 + s * ux, H + (ht - H) * x0 / L + s * uy
        b = (x + off * nx, y + off * ny)
        p_root = b
        p_peak = (b[0] + pitch * 0.15 * ux + th * nx, b[1] + pitch * 0.15 * uy + th * ny)
        p_tip = (b[0] + pitch * ux, b[1] + pitch * uy)
        p_in1 = (p_tip[0] - 0.1 * nx, p_tip[1] - 0.1 * ny)
        p_in0 = (b[0] - 0.1 * nx, b[1] - 0.1 * ny)
        out.append(Polygon([p_in0, p_root, p_peak, p_tip, p_in1]))
        s += pitch
    return out


def profile(d=DESIGN):
    L, H, ht = d["L"], d["H"], d["h_tip"]
    flat = dict(cap_style=2, join_style=2)
    parts = [LineString([(0, 0), (L, 0)]).buffer(d["t_back"] / 2, **flat),
             LineString([(0, H), (L, ht)]).buffer(d["t_front"] / 2, **flat),
             LineString([(L, 0), (L, ht)]).buffer(max(d["t_front"], d["t_back"]) / 2, **flat)]
    parts += [LineString(r).buffer(d["t_rib"] / 2, **flat) for r in rib_lines(d)]
    parts += teeth(d)
    # root block: solid from x=-root_block to x=0, spanning the back flank bottom to the front flank top
    rb = d["root_block"]
    parts.append(Polygon([(-rb, -d["t_back"] / 2), (0, -d["t_back"] / 2), (0, H + d["t_front"] / 2),
                          (-rb, H + d["t_front"] / 2)]))
    g = unary_union(parts)
    f = d["fillet"]
    g = g.buffer(f, join_style=1).buffer(-f, join_style=1)        # fillet concave corners (rib joints)
    # trim anything that grew past the original envelope ends
    g = g.intersection(box(-rb, -d["t_back"] / 2 - 0.01, L + max(d["t_front"], d["t_back"]) / 2, H + 5))
    g = g.simplify(0.02)
    return g


def bolt_center(d=DESIGN):
    return (-d["root_block"] / 2, d["H"] / 2)


if __name__ == "__main__":
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    g = profile()
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.fill(*g.exterior.xy, color="0.8")
    for h in g.interiors: ax.fill(*h.xy, color="w")
    ax.add_patch(plt.Circle(bolt_center(), DESIGN["bolt_d"] / 2, color="r"))
    ax.set_aspect("equal"); ax.grid(True)
    ax.set_title(f"TPU fin ray profile {DESIGN}", fontsize=7)
    plt.savefig("../analysis/finray_profile_v2.png", dpi=80, bbox_inches="tight")
    print("area", round(g.area, 1), "holes", len(g.interiors), "exterior pts", len(g.exterior.coords))

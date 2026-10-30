# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""HELIX DRIVE: shared rotary-to-linear module for HI5F and STAMP2.  2026-10-30, heads-cool for pm-eng.  Nothing printed or bought.

The jaw servo turns the rotor -> GT1 shaft -> HUB (= PEN1 stop dog + a DRUM r 8 with two helical grooves).  A CARRIAGE SLEEVE rides on the drum;
two 3x12 steel dowels pressed radially through the sleeve wall (at +y and -y) ride in the grooves AND in straight slots in two frame RAILS, so the
sleeve cannot turn and has to climb.  The dog swings its full 120.3 deg between the housing ribs; those ribs are the hard stops (as JG2 / SCOOP1).
OPEN stop (shaft -120.3 deg, where the swap routine parks the head) = sleeve LOWEST.  CLOSED stop (shaft 0) = sleeve STROKE higher.

Work identity  T theta = F s:  F = T eta / (r tan(alpha)),  r = 7 mm (pin contact),  tan(alpha) = s / (r theta),  eta = tan(alpha)/tan(alpha + phi),
phi = atan(0.25) (PLA on steel, assumed).  Returned by `forces()`.
TOOL frame: coupling face z = 0, tool along -z, +x = UP in the dock.  All solids are drawn in the OPEN pose (hub turned -120.3 deg) unless noted.
"""
from math import radians, degrees, atan, tan, pi, cos, sin

from build123d import Cylinder, Pos, Rot, Box

from common3 import X, outline, dock_keel, bx, INS, Axis, chamfer  # noqa

CAP = (-31.5, -34.5)
THETA_DEG = 120.3                      # GT1 jaw range = dog rib to rib
DRUM_R, CAP_BORE_R = 8.0, 8.25         # drum journal in the cap: 0.25 / side (fit.slip.pla >= 0.2)
SLV_RI, SLV_RO = 8.3, 12.0             # sleeve bore 0.3 / side on the drum
GROOVE_R0, GROOVE_HW = 6.0, 1.7        # groove floor r 6.0 (pin seats on it), width 3.4 for a 3.0 pin
PIN_R_IN, PIN_L = 6.5, 12.0            # dowel from r 6.5 to r 18.5: tip 0.5 off the groove floor (critic R1 #9); pressed until it stands 3.1 proud of the rail
PIN_HOLE_R = 3.15 / 2                  # fit.press.pla 3.05 + hole.p1s.horizontal (+0.1 over vertical): these holes print horizontal (both UNTESTED: pin-hole coupon first)
RAIL_X, RAIL_Y = (-5.0, 5.0), (12.4, 15.4)     # slot walls 3.3 (critic R1 #4: 1.8 flexed ~0.3 mm)
SLOT_HW = 1.7                          # rail slot 3.4 wide (pin 3.0)
PSI = (90.0, -90.0)                    # pins at +y and -y
WEB = (-31.0, -33.0)                   # hub web: shaft screw head seats at -33 (JG2's)
R_EFF, MU = 7.0, 0.25


class Drive:
    def __init__(self, stroke):
        self.s = stroke
        self.top_closed = CAP[1] - 0.5                       # sleeve top at the CLOSED stop (highest): 0.5 under the cap
        self.top = self.top_closed - stroke                  # OPEN pose
        self.zp = self.top - 3.6                             # pin axis, OPEN pose
        self.bot = self.zp - 5.6                             # sleeve bottom, OPEN pose
        over = stroke * 4.0 / THETA_DEG                      # groove runs 4 deg past each stop (never a stop itself)
        self.drum_bot = self.zp - over - GROOVE_HW - 2.0
        self.rail_bot = self.zp - SLOT_HW - 3.0

    # pin z for a shaft angle phi (deg, 0 = closed, -120.3 = open)
    def zpin(self, phi):
        return self.zp + self.s * (1.0 + phi / THETA_DEG)

    def forces(self, T=0.25, mu_lube=0.15):
        """F = F_t (1/tan(alpha + phi) - mu): groove friction (phi) AND the rail-slot friction on the pins (mu F_t), F_t = T / r (critic R1 #3).
        Dry PLA on steel mu 0.25 (assumed); PTFE dry lube (as on the v4 ramps) ~0.15 (assumed)."""
        th = radians(THETA_DEG)
        ta = self.s / (R_EFF * th)
        a = atan(ta)
        ft = T / (R_EFF / 1000)
        net = lambda mu: ft * (1 / tan(a + atan(mu)) - mu)
        return {"stroke_mm": round(self.s, 2), "helix_deg_from_circumferential": round(degrees(a), 1), "efficiency_est": round(net(MU) / (ft / ta), 2),
                "force_ideal_N": round(ft / ta, 1), "force_est_N": round(net(MU), 1), "force_est_lubed_N": round(net(mu_lube), 1), "torque_cap_Nm": T}

    # ------------------------------------------------------------ parts
    def hub(self, phi=-THETA_DEG):
        """PEN1 stop dog + drum with two helical grooves.  Drawn at shaft angle 0, then turned to phi."""
        h = X.pen_dog()
        z0 = X.DOG["z"][1]
        h += Pos(0, 0, (z0 + self.drum_bot) / 2 + 0.01) * Cylinder(DRUM_R, abs(z0 - self.drum_bot) + 0.02)
        h -= hexp(4.25, X.DOG["z"][0] + 0.5, -30.3)                                                    # deeper hex socket (JG2's)
        h -= Pos(0, 0, (WEB[1] + self.drum_bot) / 2 - 0.01) * Cylinder(3.4, abs(WEB[1] - self.drum_bot) + 0.02)   # screw + driver bore from below
        h -= Pos(0, 0, sum(WEB) / 2) * Cylinder(1.7, abs(WEB[0] - WEB[1]) + 1.0)
        cut = None
        n = 56
        for psi in PSI:
            for k in range(n + 1):
                ph = 4.0 - (THETA_DEG + 8.0) * k / n                       # +4 .. -124.3
                a = psi - ph                                               # hub-local angle of the pin at this pose
                z = self.zpin(ph)
                rm = (GROOVE_R0 + DRUM_R + 0.5) / 2
                c = Pos(rm * cos(radians(a)), rm * sin(radians(a)), z) * Rot(0, 0, a) * Cylinder(GROOVE_HW, DRUM_R + 0.5 - GROOVE_R0, rotation=(0, 90, 0))
                cut = c if cut is None else cut + c
        h -= cut
        return Rot(0, 0, phi) * h

    def cap(self):
        c = outline(CAP[0] + 0.01, CAP[1])
        c &= Pos(0, 0, sum(CAP) / 2) * chamfer(Box(36, 36, abs(CAP[0] - CAP[1])).edges().filter_by(Axis.Z), 1.5)
        c -= dock_keel()
        c -= bx((-40.0, -15.2), (-5.3, 5.3), (CAP[0] + 1, CAP[1] - 1))   # the cap slides UP past the keel's bottom flange (y +-5): notch it full width (path check 10/30)
        c -= Pos(0, 0, sum(CAP) / 2) * Cylinder(CAP_BORE_R, 5)
        for x, y in INS:
            c -= Pos(x, y, sum(CAP) / 2) * Cylinder(1.7, abs(CAP[0] - CAP[1]) + 0.2)
            c -= Pos(x, y, CAP[1] + 0.6 - 0.01) * Cylinder(3.4, 1.22)          # button head 5.7: 0.55 radial (screwaccess SEAT)
        return c

    def rails(self, bottom=None):
        """Two rails hanging from the cap at +-y, each with the straight pin slot."""
        zb = self.rail_bot if bottom is None else bottom
        r = None
        for s in (1, -1):
            y = tuple(sorted((s * RAIL_Y[0], s * RAIL_Y[1])))
            rr = bx(RAIL_X, y, (CAP[1] + 0.01, zb))
            rr -= bx((-SLOT_HW, SLOT_HW), (y[0] - 1, y[1] + 1), (self.zp - SLOT_HW - 1.0, self.zpin(0) + SLOT_HW + 1.0))   # 1 mm past each stop: the dog ribs stop it, never the slot ends
            r = rr if r is None else r + rr
        return r

    def sleeve(self, dz=0.0):
        """Carriage sleeve, OPEN pose (+dz)."""
        h = abs(self.top - self.bot)
        s = Pos(0, 0, (self.top + self.bot) / 2) * (Cylinder(SLV_RO, h) - Cylinder(SLV_RI, h + 1))
        for psi in PSI:
            s -= Pos(0, 0, self.zp) * Rot(0, 0, psi) * Pos(SLV_RO, 0, 0) * Cylinder(PIN_HOLE_R, 2 * (SLV_RO - SLV_RI) + 2, rotation=(0, 90, 0))
        return Pos(0, 0, dz) * s

    def pins(self, dz=0.0):
        """the two dowels as solids (OPEN pose + dz): from r PIN_R_IN outward along +-y."""
        out = None
        for psi in PSI:
            sy = 1 if psi > 0 else -1
            p = Pos(0, sy * (PIN_R_IN + PIN_L / 2), self.zp + dz) * Cylinder(1.5, PIN_L, rotation=(90, 0, 0))
            out = p if out is None else out + p
        return out


def hexp(r, z0, z1):
    from build123d import extrude, RegularPolygon
    h = extrude(RegularPolygon(radius=r, side_count=6), amount=abs(z0 - z1))
    return Pos(0, 0, z1) * Rot(0, 0, 90) * h

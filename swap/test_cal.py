# SPDX-License-Identifier: GPL-3.0-or-later
import sim_io as SI, swap_routine as SR, tcsim as T, mc, numpy as np
for seed in (5007, 1000, 1001, 1011):
    err = mc.sample(seed, "v4"); S = T.make("v4", err); io = SI.SimIO(S); SI.start_locked_at_A(S, io)
    q_seat = S["q_seat"] / SI.DEG
    f = SR.calibrate_friction(io, io.face_offset_q(q_seat, -20, 60, 0, io.read_q()))
    print(seed, "meas", f.band_deg, "true", [round(err.fric[j] / err.K[j] / SI.DEG, 2) for j in T.ARM])

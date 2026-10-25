"""Decompose the Lunar v0 tracking error (diagnostic only; the approved gate is unchanged).

python scripts/lunar_tracking_diag.py outputs/lunar_v0/diag_*_ts.json
Input: lunar_walk_eval.py --timeseries records. For each seed it reports:
- the gate's RMS (instantaneous planar velocity vs command, all steps);
- its split into steady walking (cmd_vx >= 0.75 and |cmd_wz| < 0.1 for the last 1.5 s) and transitions
  (stops, starts, turns);
- in steady walking: the slow error (1 s moving average of the velocity vs command), the within-stride oscillation
  (velocity minus its 1 s moving average, forward and lateral) and the stride period (left-foot touchdowns).
"""
import json
import sys

import numpy as np


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def analyse(rows, dt):
    a = np.asarray(rows, float)
    t, cvx, cwz, vx, vy, cl = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5]
    err = np.hypot(vx - cvx, vy)
    w = max(1, int(round(1.0 / dt)))
    k = np.ones(w) / w
    vx_lp, vy_lp = np.convolve(vx, k, "same"), np.convolve(vy, k, "same")
    look = int(round(1.5 / dt))
    walk = (cvx >= 0.75) & (np.abs(cwz) < 0.1)
    steady = np.array([walk[max(0, i - look):i + 1].all() for i in range(len(t))])
    steady[:w] = steady[-w:] = False
    td = np.flatnonzero(np.diff(cl) > 0)
    period = float(np.median(np.diff(t[td]))) if len(td) > 2 else float("nan")
    s = steady
    return {"gate_rms": round(rms(err), 3), "steady_frac": round(float(s.mean()), 2),
            "rms_steady": round(rms(err[s]), 3), "rms_transitions": round(rms(err[~s]), 3),
            "steady_slow_error": round(rms(np.hypot(vx_lp[s] - cvx[s], vy_lp[s])), 3),
            "steady_oscillation_fwd": round(rms(vx[s] - vx_lp[s]), 3),
            "steady_oscillation_lat": round(rms(vy[s] - vy_lp[s]), 3),
            "steady_mean_vx_minus_cmd": round(float(np.mean(vx[s] - cvx[s])), 3),
            "stride_period_s": round(period, 2)}


for path in sys.argv[1:]:
    d = json.load(open(path, encoding="utf-8"))
    for r in d["per_seed"]:
        rows = r["timeseries"]["rows"]
        dt = rows[1][0] - rows[0][0]
        print(path.split("/")[-1], r["seed"], json.dumps(analyse(rows, dt)))

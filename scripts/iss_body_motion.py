"""How much the G1 body swings in a recorded ISS run: pelvis rotation per phase and angular speed.

python scripts/iss_body_motion.py outputs/iss_activity/sc4_main_run.json [--until-step N] [--out ..._motion.json]
From the run record (frames at 30 Hz): pelvis angular speed (median, p95, max, share of frames > 200 deg/s) in
simulated time, and the net pelvis rotation over each phase segment (move, hold, servo, retry, ...), plus the leg
joint speed. --until-step stops at that step's capture time, to compare short tests with full runs.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run", type=Path)
    ap.add_argument("--until-step", type=int)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    rec = json.loads(args.run.read_text(encoding="utf-8"))
    F = rec["frames"]
    t = np.asarray(F["t"])
    n = len(t)
    if args.until_step:
        end = next((s["t_s"] for s in rec["segments"] if s["step"] == args.until_step and s.get("t_s")), t[-1])
        n = int(np.searchsorted(t, end)) + 1
    t = t[:n]
    rot = R.from_quat(np.asarray([q for _, q in F["pelvis"][:n]])[:, [1, 2, 3, 0]])
    w = np.degrees((rot[1:] * rot[:-1].inv()).magnitude()) / np.diff(t)
    ph = F["phase"][:n]
    segs, s0 = {}, 0
    for i in range(1, n + 1):
        if i == n or ph[i] != ph[s0]:
            if i - 1 > s0:
                segs.setdefault(ph[s0], []).append(float(np.degrees((rot[i - 1] * rot[s0].inv()).magnitude())))
            s0 = i
    legs = [k for k in F["q"][0] if any(x in k for x in ("hip", "knee", "ankle"))]
    Q = np.asarray([[f[k] for k in legs] for f in F["q"][:n]])
    qd = np.abs(np.diff(Q, axis=0)) / np.diff(t)[:, None]
    res = {"run": args.run.name, "until_step": args.until_step, "sim_s": round(float(t[-1]), 1),
           "pelvis_deg_s": {"median": round(float(np.median(w)), 1), "p95": round(float(np.percentile(w, 95)), 1),
                            "max": round(float(w.max()), 1), "frac_gt_200": round(float((w > 200).mean()), 4)},
           "net_rotation_deg_by_phase": {k: {"n": len(v), "median": round(float(np.median(v)), 1),
                                             "max": round(float(max(v)), 1)} for k, v in sorted(segs.items())},
           "leg_joint_rad_s_p95": round(float(np.percentile(qd, 95)), 3)}
    txt = json.dumps(res, indent=1)
    if args.out:
        args.out.write_text(txt, encoding="utf-8")
    print(txt)


if __name__ == "__main__":
    main()

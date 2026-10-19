"""Offline planner benchmark on failure states reconstructed from iss_activity run records (no Isaac).

Each failed run's last recorded frame (end of its last corrective replan) gives a planner request: joints, pelvis
pose, held-hand pose (as Isaac holds it), anchors from the run's spec. Every variant plans from the same states, so
results are repeatable, unlike whole closed-loop runs that branch on small state differences.

CUROBO venv: python scripts/bench_iss_planner.py --runs full_main tj3 ... --variants step_nofb step_fb traj_large
Writes outputs/iss_activity/bench_<variants>.json; "reach" = closed-chain free-hand error <= 4 cm (the capture
radius) with no sphere penetration. Execution adds tracking and contact error on top of this.
"""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plan_iss_activity as P  # noqa: E402

OUT = P.ROOT / "outputs" / "iss_activity"
VARIANTS = {
    "step_nofb": lambda r: ("step", dict(r, no_fallback=True)),     # IK step + closed-chain refinement only
    "step_fb": lambda r: ("step", r),                               # + trajopt fallback (large re-configuration)
    "traj": lambda r: ("traj_step", r),
    "traj_large": lambda r: ("traj_step", dict(r, allow_large_swing=True)),
    "traj_large_shift": lambda r: ("traj_step", dict(r, allow_large_swing=True, pelvis_shift_m=0.15)),
    "reach": lambda r: ("reach", r),                                 # panel states: IK + closed-chain refinement
    "reach_norefine": lambda r: ("reach", dict(r, no_refine=True)),
    "reach_local": lambda r: ("reach", dict(r, local_only=True)),
    "reach_large": lambda r: ("reach", dict(r, max_swing_rad=6.0)),
}


def failure_state(tag):
    run = json.loads((OUT / f"{tag}_run.json").read_text(encoding="utf-8"))
    seg = (run.get("segments") or [None])[-1]
    if not seg or seg["status"] == "CAPTURED" or not run.get("frames"):
        return None
    anchors = json.loads((OUT / f"{tag}_spec.json").read_text(encoding="utf-8"))["chain"]["anchors"]
    fr, names = run["frames"], [p.split("/")[-1] for p in run["link_paths"]]
    i, free = len(fr["t"]) - 1, seg["free_hand"]
    held = "left" if free == "right" else "right"
    pos, quat = fr["poses"][i]
    h, s = names.index(P.HANDS[held]), seg["step"]
    return {"tag": tag, "step": s, "exec_error_m": seg["hand_to_anchor_at_capture_m"], "q": fr["q"][i],
            "pelvis_pos": fr["pelvis"][i][0], "pelvis_quat_wxyz": fr["pelvis"][i][1], "held_hand": held,
            "held_pos": pos[h], "held_quat_wxyz": quat[h], "free_hand": free, "a_prev": anchors[s - 1],
            "a_cur": anchors[s], "a_next": anchors[min(s + 1, len(anchors) - 1)], "frames": 10}


def self_pen(pl, frame, st):
    """Sphere penetration of a reach pose (base placed at the measured pelvis pose)."""
    return pl.penetration(pl.qvec(frame["q"], st["pelvis_pos"], st["pelvis_quat_wxyz"]))["max_m"]


def panel_state(tag):
    """State just before the anchored panel reach (after the Handrail_B capture)."""
    run = json.loads((OUT / f"{tag}_run.json").read_text(encoding="utf-8"))
    seg = next((s for s in run.get("segments", []) if s.get("panel_reach")), None)
    if not seg or not run.get("frames"):
        return None
    fr, names = run["frames"], [p.split("/")[-1] for p in run["link_paths"]]
    i = fr["phase"].index("panel_reach") - 1
    held = seg["free_hand"]                       # the hand that just captured the rail
    pos, quat = fr["poses"][i]
    h = names.index(P.HANDS[held])
    target = json.loads((OUT / f"{tag}_spec.json").read_text(encoding="utf-8"))["chain"]["panel"]["target"]
    return {"tag": tag, "step": "panel", "exec_error_m": seg["panel_reach"]["min_distance_m"], "q": fr["q"][i],
            "pelvis_pos": fr["pelvis"][i][0], "pelvis_quat_wxyz": fr["pelvis"][i][1], "anchor_hand": held,
            "anchor_pos": pos[h], "anchor_quat_wxyz": quat[h], "reach_hand": "left" if held == "right" else "right",
            "target": target}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", nargs="+", required=True, help="run tags in outputs/iss_activity (<tag>_run.json)")
    ap.add_argument("--panel", action="store_true", help="panel-reach states instead of step failures")
    ap.add_argument("--variants", nargs="+", default=["step_nofb", "step_fb"], choices=sorted(VARIANTS))
    args = ap.parse_args()
    states = [s for s in map(panel_state if args.panel else failure_state, args.runs) if s]
    pl = P.Planner()
    rows = []
    for st in states:
        for v in args.variants:
            mode, req = VARIANTS[v](st)
            t = time.time()
            res = getattr(pl, mode)(req)
            if mode == "reach":                     # reach returns one frame with the closed-chain reach error
                res = dict(res, free_hand_error_cc_m=res["frames"][0]["reach_error_m"],
                           path_max_penetration_m=self_pen(pl, res["frames"][0], st))
            row = {"state": f'{st["tag"]}:{st["step"]}', "exec_error_m": st["exec_error_m"], "variant": v,
                   "solve_s": round(time.time() - t, 1), "cc_error_m": res.get("free_hand_error_cc_m"),
                   "penetration_m": res.get("path_max_penetration_m"), "plan_success": res.get("plan_success"),
                   "fallback": res.get("fallback")}
            row["reach"] = bool(row["cc_error_m"] is not None and row["cc_error_m"] <= 0.04
                                and (row["penetration_m"] or 0.0) <= 0.0)
            rows.append(row)
            print(json.dumps(row), flush=True)
    summary = {v: {"reach": sum(r["reach"] for r in rows if r["variant"] == v), "states": len(states)}
               for v in args.variants}
    (OUT / f"bench_{'panel_' if args.panel else ''}{'_'.join(args.variants)}.json").write_text(json.dumps({"summary": summary, "rows": rows}, indent=1),
                                                              encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()

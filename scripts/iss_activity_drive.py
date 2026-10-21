"""Run the full-ISS G1 hand-over-hand traverse end to end (planner service + Isaac PhysX run + interference check).

CUROBO_PYTHON=<cuRobo V2 venv python> python scripts/iss_activity_drive.py [--port 47815] [--max-retries 4]
Prerequisites: scenes/iss_full_v0.usda (scripts/build_iss_full.py, route anchors in its build_report.json) and
outputs/iss_full_v0/world/ (scripts/export_iss_world.py).
Routes (--route): main = Node1 -> US Lab -> Node2 -> Columbus along the route anchors (hatch rims, the S1 Handrail_B, PLACEHOLDER
handholds); anchored panel reach after Handrail_B. Outputs: outputs/iss_activity/ (gitignored): activity_spec.json,
<tag>_run.json (+ _g1 import, log), <tag>_check.json (per-link penetration), <tag>_summary.json.
Contacts are ON by default (PhysX resolves G1 vs ISS collisions; contact events are counted per link).
History: ballistic push-off/flight/capture between sparse anchors was tried first and never captured (residual
release spin of the floppy-legged G1 over 10-20 s flights); see docs/ISS_TO_LUNAR_SHOWCASE_ARCHITECTURE.md.
"""
import argparse
import json
import os
import socket
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "iss_activity"
ISAAC = os.environ.get("ISAAC_PYTHON", r"C:\isaacsim\python.bat")
CONTROL = {"gains": [300.0, 30.0], "frames": 10, "duration_s": 4.0, "hold_s": 1.0, "settle_s": 0.4,
           "retry_duration_s": 2.5}


def ask(port, req):
    with socket.create_connection(("127.0.0.1", port)) as s, s.makefile("rw", encoding="utf-8") as fh:
        fh.write(json.dumps(req) + "\n")
        fh.flush()
        return json.loads(fh.readline())


def contact_totals(segs):
    """PhysX contact-report events (G1 link vs ISS collider, per physics step) summed per link over the run."""
    tot = {}
    for s in segs:
        for k, v in (s.get("contacts") or {}).items():
            link = k.split(":", 1)[1]
            tot[link] = tot.get(link, 0) + v
    return dict(sorted(tot.items(), key=lambda x: -x[1]))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=47815)
    ap.add_argument("--max-retries", type=int, default=6)
    ap.add_argument("--filter-contacts", action="store_true",
                    help="collision-filter G1 vs ISS (S1.3/S1.4 condition); default: contacts ON, resolved by PhysX")
    ap.add_argument("--tag", default="activity", help="output file prefix")
    ap.add_argument("--route", default="main", help="scenario route in build_report.json routes: main (Node1 -> US Lab "
                    "-> Node2 -> Columbus, with the panel reach), jem (Node2 -> JEM PM), node3 (Node1 -> Node3)")
    ap.add_argument("--gains", type=float, nargs=2, default=CONTROL["gains"], metavar=("KP", "KD"))
    ap.add_argument("--velocity-ff", action="store_true", help="joint velocity targets = d(position target)/dt")
    ap.add_argument("--time-scale", type=float, default=1.0, help="multiplier on every move/retry duration")
    ap.add_argument("--hold-s", type=float, default=CONTROL["hold_s"])
    ap.add_argument("--retry-mode", default="step", choices=["step", "traj_step"],
                    help="planner mode for corrective replans")
    ap.add_argument("--hand-servo", action="store_true", help="closed-loop free-hand correction after each move")
    ap.add_argument("--gravity", type=float, default=0.0, help="scene gravity magnitude (m/s^2, -z); 9.81 = ground")
    ap.add_argument("--max-steps", type=int, help="stop after this many steps (ablations)")
    ap.add_argument("--planner-mode", default="traj_step", choices=["traj_step", "step"],
                    help="traj_step: cuRobo trajectory optimisation with the held hand constrained along the path "
                         "(default); step: chained IK waypoints")
    args = ap.parse_args()
    py = os.environ.get("CUROBO_PYTHON")
    if not py:
        raise SystemExit("set CUROBO_PYTHON to the cuRobo V2 venv python")
    OUT.mkdir(parents=True, exist_ok=True)
    rep = json.loads((ROOT / "outputs/iss_full_v0/build_report.json").read_text(encoding="utf-8"))
    route = rep["routes"][args.route]
    anchors = [a["pos_scene_m"] for a in route["anchors"]]
    world = json.loads((ROOT / "outputs/iss_full_v0/world/anchors.json").read_text(encoding="utf-8"))
    panel = world["targets"][0]["centre_m"]
    rail_b = next((a["index"] for a in route["anchors"] if a.get("panel_station")),
                  next((a["index"] for a in route["anchors"] if a["kind"] == "graspable_rail"), None))   # reach anchor

    log = open(OUT / "planner_service.log", "w", encoding="utf-8")
    srv = subprocess.Popen([py, str(ROOT / "scripts/plan_iss_activity.py"), "--serve", str(args.port), "--frame-offset",
                            *map(str, route["frame_offset_m"])], cwd=ROOT,
                           stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
    check = None
    try:
        for _ in range(180):
            if "serving" in (OUT / "planner_service.log").read_text(encoding="utf-8", errors="replace"):
                break
            time.sleep(1)
        else:
            raise SystemExit("planner service did not start")
        st = ask(args.port, {"mode": "start", "pelvis_pos": route["start_pelvis_m"], "facing": route["facing"],
                             "anchor_hand": "right", "anchor_point": anchors[0]})["frames"][0]
        spec = {"chain": {"start_state": {"q": st["q"], "pelvis_pos": st["pelvis_pos"],
                                          "pelvis_quat_wxyz": st["pelvis_quat_wxyz"],
                                          "anchor": {"hand": "right", **st["hands"]["right"]}},
                          "start_plan": {k: st[k] for k in ("success", "feasible", "hand_goal_error_m")},
                          "anchors": anchors, "planner_port": args.port, "max_retries": args.max_retries,
                          "planner_mode": args.planner_mode, "retry_mode": args.retry_mode,
                          "panel": ({"after_anchor": rail_b, "target": [panel[0], panel[1], panel[2] - 0.06]}
                                    if rail_b is not None else None),
                          "route": args.route, "max_steps": args.max_steps,
                          **CONTROL, "gains": args.gains, "hold_s": args.hold_s,
                          "velocity_feedforward": args.velocity_ff, "hand_servo": args.hand_servo, "gravity_m_s2": args.gravity, "time_scale": args.time_scale}}
        (OUT / (args.tag + "_spec.json")).write_text(json.dumps(spec), encoding="utf-8")
        cmd = (f'"{ISAAC}" scripts\\iss_activity_run.py --spec "{OUT / (args.tag + "_spec.json")}" '
               f'--out "{OUT / (args.tag + "_run.json")}" --record' + (" --filter-contacts" if args.filter_contacts else ""))
        with open(OUT / (args.tag + "_run.log"), "w", encoding="utf-8") as rlog:
            subprocess.run(cmd, cwd=ROOT, shell=True, stdout=rlog, stderr=subprocess.STDOUT)
        run = json.loads((OUT / (args.tag + "_run.json")).read_text(encoding="utf-8"))
        if run.get("frames"):
            check = ask(args.port, {"mode": "check", "run": str(OUT / (args.tag + "_run.json")), "every": 3})
            (OUT / (args.tag + "_check.json")).write_text(json.dumps(check, indent=1), encoding="utf-8")
    finally:
        srv.terminate()
        log.close()
    segs = run.get("segments", [])
    summary = {"route": args.route, "control": run.get("control"), "error": run.get("error"), "completed_steps": run.get("completed_steps"),
               "total_steps": run.get("total_steps"), "sim_time_s": run.get("sim_time_s"),
               "last_pelvis_pos": next((s["pelvis_pos"] for s in reversed(segs) if s.get("pelvis_pos")), None),
               "capture_error_m": [round(s["hand_to_anchor_at_capture_m"], 4) for s in segs],
               "corrective_replans": [s.get("corrective_replans") for s in segs],
               "hand_to_anchor_after_move_m": [round(s["hand_to_anchor_after_move_m"], 3) for s in segs],
               "max_joint_error_rad": [round(s["move_tracking"]["max_joint_error_rad"], 3) for s in segs],
               "pelvis_error_m": [s["move_tracking"]["pelvis_error_m"] and round(s["move_tracking"]["pelvis_error_m"], 3)
                                  for s in segs],
               "drive_saturation_fraction": [round(s["drive_saturation"]["fraction_of_samples"], 3) for s in segs],
               "contact_events_by_step": [sum((s.get("contacts") or {}).values()) for s in segs],
               "g1_mass_kg": run.get("g1_mass_kg"),
               "loads_by_step": [s.get("loads") for s in segs],
               "panel_reach": next((s["panel_reach"] for s in segs if s.get("panel_reach")), None),
               "contacts_filtered": run.get("contacts_filtered"),
               "contact_events_by_link": contact_totals(segs),
               "penetration": {k: check[k] for k in ("sampled", "max_penetration_m", "frames_penetrating")} if check else None}
    (OUT / (args.tag + "_summary.json")).write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()

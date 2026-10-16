"""cuRobo planner for the full-ISS G1 hand-over-hand traverse (kinematic, collision-aware; runs OUTSIDE Isaac).

Run with the cuRobo V2 venv (NVlabs/curobo @ 78fd485, Apache-2.0), not with Isaac's python:
  <curobo-venv>\\Scripts\\python.exe scripts/plan_iss_activity.py --serve PORT          (service for iss_activity_run.py)
  <curobo-venv>\\Scripts\\python.exe scripts/plan_iss_activity.py --request req.json --out plan.json
World: outputs/iss_full_v0/world/{collision_world.npz, anchors.json} from scripts/export_iss_world.py (scene frame).
Robot: cuRobo's unitree_g1_29dof_retarget.yml (g1_29dof_rev_1_0 + a 6-joint virtual floating base), tool frames
reduced to pelvis + both rubber hands, legs locked in a tuck. Same 29 joints and inertials as the canonical Isaac
asset (verified by URDF diff; cuRobo adds base_link and differs only in the mid360 sensor frame).
Modes: start (anchored start pose), step (one hand-over-hand step from a measured state), chain (open-loop preview
of the whole route), reach (anchored free-hand reach), check (post-hoc interference of a recorded run).
cuRobo is a kinematic planner: dynamics and all verdicts come from the Isaac PhysX run. G1 self-collision is not
checked (cuRobo's G1 sphere model self-collides at q = 0; Isaac also runs without self-collision).
"""
import argparse
import json
import time
from importlib.resources import files
from pathlib import Path

import numpy as np
import torch
import yaml
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.scene import Mesh, Scene
from curobo.types import GoalToolPose, Pose
from curobo._src.cost.tool_pose_criteria import ToolPoseCriteria
from curobo._src.state.state_joint import JointState
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "outputs" / "iss_full_v0" / "world"
OFFSET = np.array([3.0, 0.0, -4.84])      # planning frame = RP * (scene - OFFSET); base prisms are limited to +-10 m
# RP maps scene +x (station forward) to planning +z, so head-first travel along the station is near the identity
# orientation: the floating base chain X_ROT -> Y_ROT -> Z_ROT is singular at Y = +-90 deg (gimbal lock).
RP = Rotation.from_euler("y", -90.0, degrees=True)
def wxyz(r):
    q = r.as_quat()
    return [q[3], q[0], q[1], q[2]]


def rot(qwxyz):
    return Rotation.from_quat(np.r_[qwxyz[1:], qwxyz[0]])


def to_plan_pt(p):
    return RP.apply(np.asarray(p, float) - OFFSET)


def from_plan_pt(p):
    return RP.inv().apply(np.asarray(p, float)) + OFFSET


def base_seed(pelvis_pos, pelvis_rot):
    """Base joint values (planning frame) for a scene-frame pelvis position and Rotation."""
    return dict(zip(BASE, list(to_plan_pt(pelvis_pos)) + list((RP * pelvis_rot).as_euler("XYZ"))))


HANDS = {"left": "left_rubber_hand", "right": "right_rubber_hand"}
BASE = ["base_j_x", "base_j_y", "base_j_z", "base_j_xtheta", "base_j_ytheta", "base_j_ztheta"]


# legs locked in a tuck (crew tuck their legs in IVA translation; straight legs swept the hatchways in Isaac)
LEG_TUCK = {f"{s}_{j}_joint": v for s in ("left", "right")
            for j, v in (("hip_pitch", -0.6), ("hip_roll", 0.0), ("hip_yaw", 0.0), ("knee", 1.2),
                         ("ankle_pitch", -0.6), ("ankle_roll", 0.0))}


def robot_cfg():
    cfg = yaml.safe_load(open(files("curobo") / "content" / "configs" / "robot" / "unitree_g1_29dof_retarget.yml"))
    cfg["kinematics"]["tool_frames"] = ["pelvis", HANDS["left"], HANDS["right"]]
    cfg["kinematics"]["lock_joints"] = dict(LEG_TUCK)
    return cfg


def scene():
    w = np.load(WORLD / "collision_world.npz")
    meshes = []
    for k in w.files:
        tri = to_plan_pt(w[k].reshape(-1, 3))
        meshes.append(Mesh(name=k.split("|")[-1][:40] + f"_{len(meshes)}", pose=[0, 0, 0, 1, 0, 0, 0],
                           vertices=tri.tolist(), faces=np.arange(len(tri)).tolist()))
    return Scene(mesh=meshes)


def T(x):
    return torch.tensor(np.asarray(x, dtype=np.float32)[None], device="cuda")


def facing_quat(d):
    """Head-first travel pose: body +z (head) along d, body +x (chest) as close to scene +z (zenith) as possible.

    G1 is ~1.3 m tall and ISS hatches are ~1.26 m square, so it cannot pass upright; crew traverse head-first.
    """
    z = np.asarray(d, float) / np.linalg.norm(d)
    up = np.array([0, 0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0, 0])
    x = up - np.dot(up, z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    q = Rotation.from_matrix(np.c_[x, y, z]).as_quat()      # xyzw
    return [q[3], q[0], q[1], q[2]]


class Planner:
    def __init__(self, use_scene=True):
        cfg = robot_cfg()
        self.scene = scene() if use_scene else None
        # self-collision OFF: cuRobo's G1 sphere model self-collides at q = 0 (constraint value 2.0 with the shipped
        # retarget config); the Isaac executor also runs G1 without self-collision (gate condition). World collision ON.
        # use_cuda_graph=False: graph replay crashed (CUDA illegal address) after the per-request tool-pose criteria
        # and seed shapes changed; graphs are only a speed optimisation
        common = dict(robot=cfg, scene_model=self.scene, self_collision_check=False, position_tolerance=0.01,
                      use_cuda_graph=False,
                      orientation_tolerance=3.2)
        self.ik = InverseKinematics(InverseKinematicsCfg.create(num_seeds=64, **common))
        self.ik_local = InverseKinematics(InverseKinematicsCfg.create(num_seeds=8, **common))
        self.joints = list(self.ik.joint_names)
        self.gen = torch.Generator(device="cuda").manual_seed(0)   # deterministic seed perturbations

    def criteria(self, anchor_hand, reach_hand, anchor_ori):
        c = {"pelvis": ToolPoseCriteria.track_position_and_orientation(xyz=[0.1] * 3, rpy=[0.3] * 3)}
        c[HANDS[anchor_hand]] = (ToolPoseCriteria.track_position_and_orientation(xyz=[1.0] * 3, rpy=[1.0] * 3)
                                 if anchor_ori else ToolPoseCriteria.track_position(xyz=[1.0] * 3))
        c[HANDS[reach_hand]] = ToolPoseCriteria.track_position(xyz=[0.05] * 3)
        for s in (self.ik, self.ik_local):
            s.update_tool_pose_criteria(c)

    def fk(self, qvec):
        ks = self.ik.compute_kinematics(JointState.from_position(torch.as_tensor(qvec, device="cuda")[None],
                                                                joint_names=self.joints))
        out = {}
        for name in ["pelvis", HANDS["left"], HANDS["right"]]:
            i = self.ik.tool_frames.index(name)
            n = len(self.ik.tool_frames)
            qp = ks.tool_poses.quaternion.reshape(-1, n, 4)[0, i].cpu().numpy()
            out[name] = (from_plan_pt(ks.tool_poses.position.reshape(-1, n, 3)[0, i].cpu().numpy()),
                         np.asarray(wxyz(RP.inv() * rot(qp))))
        return out

    def solve(self, goals, solver, seed=None):
        g = {k: Pose(position=T(to_plan_pt(p)), quaternion=T(wxyz(RP * rot(q)))) for k, (p, q) in goals.items()}
        seeds = None
        if seed is not None:
            # distinct seeds: the given seed, then small perturbations of it (identical seeds waste the batch)
            s0 = torch.as_tensor(np.asarray(seed, np.float32), device="cuda")
            n = solver.config.num_seeds
            noise = torch.randn((n, s0.numel()), device="cuda", generator=self.gen) * 0.05
            noise[0] = 0.0
            seeds = (s0[None] + noise).view(1, n, -1)
        solver.reset_seed()
        self.gen.manual_seed(0)
        res = solver.solve_pose(GoalToolPose.from_poses(g, ordered_tool_frames=solver.tool_frames), seed_config=seeds)
        q = self.active(res.js_solution)[0]
        return q, self.constraints(torch.as_tensor(q, device="cuda").view(1, 1, -1))

    def active(self, js):
        """(n, active dof) array from a solver joint state, which also carries the locked (tucked leg) joints."""
        pos = js.position.reshape(-1, js.position.shape[-1]).cpu().numpy()
        idx = [list(js.joint_names).index(j) for j in self.joints]
        return pos[:, idx]

    def solve_candidates(self, goals, k=8):
        """Global multi-seed IK returning the k best joint vectors (planning frame)."""
        g = {n: Pose(position=T(to_plan_pt(p)), quaternion=T(wxyz(RP * rot(q)))) for n, (p, q) in goals.items()}
        self.ik.reset_seed()                               # deterministic: same request -> same candidates
        res = self.ik.solve_pose(GoalToolPose.from_poses(g, ordered_tool_frames=self.ik.tool_frames), return_seeds=k)
        return list(self.active(res.js_solution))

    def constraints(self, qt):
        """Constraint values of a configuration, excluding self-collision (always violated by cuRobo's G1 sphere
        model at q = 0, see __init__). Feasible = every remaining constraint (joint limits, world collision) is 0."""
        b = self.ik_local.config.num_seeds         # rollout buffers are sized for the solver's seed batch
        cc = self.ik_local.metrics_rollout.compute_metrics_from_action(qt.repeat(b, 1, 1)).costs_and_constraints.constraints
        vals = {n: float(v[0].max()) for n, v in zip(cc.names, cc.values) if n != "self_collision"}
        return {"feasible": all(v <= 0.0 for v in vals.values()), "constraints": vals}

    def frame(self, q, feasible, goals, anchor):
        """success = cuRobo feasibility (joint limits + ISS world collision; self-collision not checked) and anchored hand within 1 cm."""
        k = self.fk(q)
        err = {n: float(np.linalg.norm(k[n][0] - np.asarray(goals[n][0]))) for n in goals if n != "pelvis"}
        ok = feasible["feasible"] and err[HANDS[anchor]] <= 0.01
        return {"q": {**LEG_TUCK, **{j: float(v) for j, v in zip(self.joints, q) if j not in BASE}},
                "base": {j: float(v) for j, v in zip(self.joints, q) if j in BASE},
                "pelvis_pos": k["pelvis"][0].tolist(), "pelvis_quat_wxyz": k["pelvis"][1].tolist(),
                "hands": {h: {"pos": k[HANDS[h]][0].tolist(), "quat_wxyz": k[HANDS[h]][1].tolist()} for h in HANDS},
                "hand_goal_error_m": err, "feasible": feasible["feasible"],
                "constraints": feasible["constraints"], "success": ok}

    def start(self, r):
        self.criteria(r["anchor_hand"], "left" if r["anchor_hand"] == "right" else "right", anchor_ori=False)
        fq = facing_quat(r["facing"])
        reach = "left" if r["anchor_hand"] == "right" else "right"
        p = np.asarray(r["pelvis_pos"], float)
        goals = {"pelvis": (p, fq), HANDS[r["anchor_hand"]]: (r["anchor_point"], fq),
                 HANDS[reach]: (p + 0.45 * np.asarray(r["facing"]) / np.linalg.norm(r["facing"]), fq)}
        # seed: neutral joints (q = 0) with the base at the requested pelvis pose (avoids contorted global optima)
        base = base_seed(p, rot(fq))
        seed = np.array([base.get(j, 0.0) for j in self.joints], dtype=np.float32)
        q, ok = self.solve(goals, self.ik_local, seed)
        return {"frames": [self.frame(q, ok, goals, r["anchor_hand"])]}

    def reach(self, r):
        """Anchored reach: anchored hand pose fixed, free hand to a target point, pelvis near its current pose."""
        anchor, hand = r["anchor_hand"], r["reach_hand"]
        c = {"pelvis": ToolPoseCriteria.track_position_and_orientation(xyz=[0.05] * 3, rpy=[0.05] * 3),
             HANDS[anchor]: ToolPoseCriteria.track_position_and_orientation(xyz=[1.0] * 3, rpy=[1.0] * 3),
             HANDS[hand]: ToolPoseCriteria.track_position(xyz=[1.0] * 3)}
        for s in (self.ik, self.ik_local):
            s.update_tool_pose_criteria(c)
        base = base_seed(r["pelvis_pos"], rot(r["pelvis_quat_wxyz"]))
        seed = np.array([base[j] if j in base else r["q"].get(j, 0.0) for j in self.joints], dtype=np.float32)
        goals = {"pelvis": (r["pelvis_pos"], r["pelvis_quat_wxyz"]),
                 HANDS[anchor]: (r["anchor_pos"], r["anchor_quat_wxyz"]),
                 HANDS[hand]: (r["target"], r["pelvis_quat_wxyz"])}
        q, ok = self.solve(goals, self.ik_local, seed)
        f = self.frame(q, ok, goals, anchor)
        f["reach_error_m"] = f["hand_goal_error_m"][HANDS[hand]]
        return {"frames": [f]}


    def plan_step(self, q, held, held_pose, free, a_prev, a_cur, a_next, frames_n):
        """One hand-over-hand step: held hand fixed at held_pose, free hand to a_cur.

        (1) final pose = global multi-seed IK candidate that is feasible (held hand <= 1 cm, free hand <= 4 cm =
        the Isaac capture radius, no world collision) and closest to the current joints; otherwise the feasible
        candidate with the smallest free-hand error (step marked failed). (2) execution path = actuated joints
        interpolated toward it (Isaac holds the anchored hand with a fixed joint; only the final pose is checked).
        """
        route = a_next - a_prev
        route = route / np.linalg.norm(route) if np.linalg.norm(route) > 1e-6 else np.array([1.0, 0, 0])
        # pelvis orientation goal: 30 % of the way from the current orientation toward head-first along the route
        # (a full re-orientation per step demands large rotations about the fixed hand that joint PD cannot follow)
        k_now = self.fk(q)
        fq = wxyz(Slerp([0, 1], Rotation.concatenate([rot(k_now["pelvis"][1]), rot(facing_quat(route))]))(0.3))
        # pelvis weight 0.3 (was 0.05): a weak pelvis term let the body creep toward the overhead anchors step by step
        c = {"pelvis": ToolPoseCriteria.track_position_and_orientation(xyz=[0.3] * 3, rpy=[0.1] * 3),
             HANDS[held]: ToolPoseCriteria.track_position_and_orientation(xyz=[1.0] * 3, rpy=[1.0] * 3),
             HANDS[free]: ToolPoseCriteria.track_position(xyz=[1.0] * 3)}
        for s in (self.ik, self.ik_local):
            s.update_tool_pose_criteria(c)
        pel_goal = (a_prev + a_cur) / 2 + np.array([0, 0, -0.55])
        goals_f = {"pelvis": (pel_goal, fq), HANDS[held]: held_pose, HANDS[free]: (a_cur, fq)}
        q_cur, best, fallback = q.copy(), None, None
        act_i = [i for i, j in enumerate(self.joints) if j not in BASE]
        # first: warm-started local IK from the current joints (smallest motion when it converges)
        ql, okl = self.solve(goals_f, self.ik_local, q_cur)
        fl = self.frame(ql, okl, goals_f, held)
        if fl["success"] and fl["hand_goal_error_m"][HANDS[free]] <= 0.04:
            best = (float(np.abs(ql - q_cur)[act_i].max()), ql)
        for kk in (() if best is not None else (8, 32)):
            for qc in self.solve_candidates(goals_f, k=kk):
                fc = self.frame(qc, self.constraints(torch.as_tensor(qc, device="cuda").view(1, 1, -1)), goals_f, held)
                err = fc["hand_goal_error_m"][HANDS[free]]
                dq = float(np.abs(qc - q_cur)[act_i].max())
                if fc["success"] and err <= 0.04 and dq <= 0.8:     # large joint swings do not execute on PD
                    if best is None or dq < best[0]:
                        best = (dq, qc)
                elif fc["feasible"] and err <= 0.04 and (fallback is None or dq < fallback[0]):
                    fallback = (dq, qc)                          # reachable but large swing: smallest swing
            if best is not None:
                break
        q_final = best[1] if best is not None else (fallback[1] if fallback else q_cur)
        f_final = self.frame(q_final, self.constraints(torch.as_tensor(q_final, device="cuda").view(1, 1, -1)), goals_f, held)
        # execution path: per frame, local IK (held hand fixed, free hand and pelvis interpolated) seeded by the
        # joint-space interpolation; the interpolation itself is used only where that IK fails (closed-chain drift)
        act = np.array([j not in BASE for j in self.joints])
        k0, kf = self.fk(q_cur), self.fk(q_final)
        frames, consistent = [], 0
        for n in range(1, frames_n):
            u = n / frames_n
            qi = q_final.copy()
            qi[act] = q_cur[act] + u * (q_final[act] - q_cur[act])
            hand = k0[HANDS[free]][0] + u * (kf[HANDS[free]][0] - k0[HANDS[free]][0])
            pel = k0["pelvis"][0] + u * (kf["pelvis"][0] - k0["pelvis"][0])
            goals = {"pelvis": (pel, wxyz(rot(kf["pelvis"][1]))), HANDS[held]: held_pose, HANDS[free]: (hand, fq)}
            qn, ok = self.solve(goals, self.ik_local, qi)
            fn = self.frame(qn, ok, goals, held)
            if fn["success"] and float(np.abs(qn - qi)[act].max()) <= 0.5:
                frames.append(fn)
                consistent += 1
            else:
                frames.append({"q": {**LEG_TUCK, **{j: float(v) for j, v in zip(self.joints, qi) if j not in BASE}}})
        frames.append(f_final)
        free_err = f_final["hand_goal_error_m"][HANDS[free]]
        return {"free_hand": free, "held_hand": held, "anchor": np.asarray(a_cur).tolist(), "frames": frames,
                "free_hand_final_error_m": free_err, "final_ik_found": best is not None,
                "final_collision_free": f_final["feasible"], "path_consistent_frames": f"{consistent}/{frames_n - 1}",
                "path_check": "IK-consistent frames are collision-checked; interpolated fallbacks are not",
                "plan_success": best is not None and free_err <= 0.04}, q_final

    def qvec(self, qdict, pelvis_pos, pelvis_quat):
        base = base_seed(pelvis_pos, rot(pelvis_quat))
        return np.array([base[j] if j in base else qdict.get(j, 0.0) for j in self.joints], np.float32)

    def step(self, r):
        """Closed-loop step from the measured Isaac state: {"mode": "step", "q", "pelvis_pos", "pelvis_quat_wxyz",
        "held_hand", "held_pos", "held_quat_wxyz", "free_hand", "a_prev", "a_cur", "a_next", "frames"}."""
        q = self.qvec(r["q"], r["pelvis_pos"], r["pelvis_quat_wxyz"])
        st, _ = self.plan_step(q, r["held_hand"], (r["held_pos"], r["held_quat_wxyz"]), r["free_hand"],
                               np.asarray(r["a_prev"], float), np.asarray(r["a_cur"], float),
                               np.asarray(r["a_next"], float), r["frames"])
        return dict(st, frames=st["frames"])

    def chain(self, r):
        """Open-loop kinematic plan of the whole hand-over-hand traverse (diagnostic / preview; the executed run
        replans every step from the measured state via the --serve planner). Request: {"mode": "chain", "start":
        <start frame>, "start_hand", "anchors": [[x,y,z], ...], "frames": n}."""
        anchors = [np.asarray(a, float) for a in r["anchors"]]
        cur = r["start"]
        q = np.array([cur["base"].get(j, 0.0) if j in BASE else cur["q"].get(j, 0.0) for j in self.joints], np.float32)
        held = r["start_hand"]
        steps = []
        for i in range(1, len(anchors)):
            free = "left" if held == "right" else "right"
            k = self.fk(q)
            st, q = self.plan_step(q, held, (k[HANDS[held]][0], wxyz(rot(k[HANDS[held]][1]))), free, anchors[i - 1],
                                   anchors[i], anchors[min(i + 1, len(anchors) - 1)], r["frames"])
            st["step"] = i
            steps.append(st)
            print("chain step", i, free, "ok", st["plan_success"], "free err", round(st["free_hand_final_error_m"], 4), flush=True)
            held = free
        return {"frames": [s["frames"][-1] for s in steps], "steps": steps,
                "all_steps_ok": all(s["plan_success"] for s in steps)}

    def check(self, r):
        """Post-hoc interference of executed poses: {"mode": "check", "run": <iss_activity_run.py json with frames>}.
        Per frame: cuRobo sphere model (legs at the recorded joints are NOT represented: cuRobo locks them at the
        tuck) against the ISS world; returns frames with scene_collision > 0."""
        run = json.loads(Path(r["run"]).read_text(encoding="utf-8"))
        f = run["frames"]
        hits, worst = [], 0.0
        for i, ((pp, pq), q) in enumerate(zip(f["pelvis"], f["q"])):
            base = base_seed(pp, rot(pq))
            qv = np.array([base[j] if j in base else q.get(j, 0.0) for j in self.joints], np.float32)
            c = self.constraints(torch.as_tensor(qv, device="cuda").view(1, 1, -1))["constraints"].get("scene_collision", 0.0)
            if c > 0:
                hits.append({"t_s": f["t"][i], "phase": f["phase"][i], "cost": c})
                worst = max(worst, c)
        return {"frames": [], "checked": len(f["t"]), "interference_frames": len(hits), "max_cost": worst,
                "hits": hits[:200], "note": "cuRobo scene_collision > 0 = sphere model within activation distance "
                                            "of or penetrating the ISS meshes (legs at the planner tuck)"}


def serve(port, pl):
    """Local planner service for closed-loop execution: one JSON request per line in, one JSON result per line out."""
    import socket
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    print(f"serving on 127.0.0.1:{port}", flush=True)
    while True:
        conn, _ = srv.accept()
        try:
            handle(conn, pl)
        except (ConnectionError, OSError) as exc:            # a dropped client must not stop the service
            print(f"connection dropped: {exc}", flush=True)
        except StopIteration:
            return


def handle(conn, pl):
    if True:
        with conn, conn.makefile("rw", encoding="utf-8") as fh:
            for line in fh:
                if not line.strip():
                    continue
                try:
                    r = json.loads(line)
                except json.JSONDecodeError as exc:            # malformed client line: answer, keep serving
                    fh.write(json.dumps({"error": f"bad request: {exc}"}) + "\n")
                    fh.flush()
                    continue
                if r.get("mode") == "shutdown":
                    raise StopIteration
                t1 = time.time()
                try:
                    res = getattr(pl, r["mode"])(r)
                    res["solve_s"] = round(time.time() - t1, 2)
                except Exception as exc:                  # report, keep serving
                    res = {"error": f"{type(exc).__name__}: {exc}"}
                fh.write(json.dumps(res) + "\n")
                fh.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--request", type=Path)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--no-scene", action="store_true", help="diagnostic: plan without the ISS collision world")
    ap.add_argument("--serve", type=int, help="serve requests on 127.0.0.1:PORT (one JSON per line) instead")
    args = ap.parse_args()
    if args.serve:
        serve(args.serve, Planner(use_scene=not args.no_scene))
        return
    reqs = json.loads(args.request.read_text(encoding="utf-8"))
    t0 = time.time()
    pl = Planner(use_scene=not args.no_scene)
    out = {"planner": "cuRobo V2 (NVlabs/curobo@78fd485)", "offset": OFFSET.tolist(), "plan_frame_rotation": "Ry(-90 deg)", "init_s": round(time.time() - t0, 1),
           "results": []}
    for r in (reqs if isinstance(reqs, list) else [reqs]):
        t1 = time.time()
        res = getattr(pl, r["mode"])(r)
        res.update({"mode": r["mode"], "request": r, "solve_s": round(time.time() - t1, 2),
                    "all_success": all(f["success"] for f in res["frames"])})
        out["results"].append(res)
        if "anchor_hand" in r:
            print(r["mode"], "success", res["all_success"], "feasible", [f["feasible"] for f in res["frames"]][:3],
                  "anchor err max", max(f["hand_goal_error_m"][HANDS[r["anchor_hand"]]] for f in res["frames"]), flush=True)
        else:
            print(r["mode"], "steps ok", res.get("all_steps_ok"), flush=True)
    args.out.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

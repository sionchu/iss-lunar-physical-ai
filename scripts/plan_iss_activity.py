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
checked by default (cuRobo's shipped G1 sphere model overlaps at rest); --self-collision turns the check on with
the rest-pose overlaps resolved in robot_cfg() (see SELF_IGNORE / SELF_BUFFER), for runs with PhysX self-collision on.
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


LARGE_SWING_RAD = 6.0  # trajopt re-configuration seed limit when no small-swing step exists (main step 21: 5.6 rad)
MAX_SWING_RAD = 1.5    # largest actuated-joint change per step; executed slowly (SECONDS_PER_RAD), see step()
SECONDS_PER_RAD = 3.0
HELD_TOL_M = 0.02     # planner acceptance for the anchored hand (the fixed joint holds it in execution); the
                      # warm-started solver lands at 1.1-1.6 cm, so 1 cm forced large global swings or no plan
TRAJ_TIME_SCALE = 1.0  # trajectory time multiplier for joint-PD tracking
CLEARANCE_M = 0.04   # world-collision activation distance: plans keep this margin (execution deviates from plans)
HELD_RPY_W = 1.0     # held-hand orientation weight in IK; 5 (to cut the planned held-hand rotation) lost reachability:
                     # 2 vs 6 of 12 steps (ab_cc3 vs ab_cc2); the closed-chain refinement corrects the rotation instead
CC_TOL_M = 0.015     # closed-chain refinement target: free hand placed through the measured held-hand pose
CENTRE_Z = -4.84     # shared module axis height in the scene frame (measured hatch centres)
HANDS = {"left": "left_rubber_hand", "right": "right_rubber_hand"}
BASE = ["base_j_x", "base_j_y", "base_j_z", "base_j_xtheta", "base_j_ytheta", "base_j_ztheta"]


# legs locked in a tuck (crew tuck their legs in IVA translation; straight legs swept the hatchways in Isaac)
LEG_TUCK = {f"{s}_{j}_joint": v for s in ("left", "right")
            for j, v in (("hip_pitch", -0.6), ("hip_roll", 0.0), ("hip_yaw", 0.0), ("knee", 1.2),
                         ("ankle_pitch", -0.6), ("ankle_roll", 0.0))}


# Rest-pose (actuated 0, legs tucked) overlaps of the shipped sphere model, from scripts/g1_self_collision.py:
# shoulder_roll|torso 6 mm and wrist_roll|wrist_yaw (one compact wrist, two joints apart) are proxy artefacts of
# links that sit against each other by construction, so they are ignored; shoulder_yaw|torso (7 mm at rest) can
# really collide when the arm adducts, so its spheres shrink by 1 cm for self-collision instead of being ignored.
SELF_IGNORE = {f"{s}_{a}": [f"{s}_{b}" if b.startswith("wrist") else b] for s in ("left", "right")
               for a, b in (("shoulder_roll_link", "torso_link"), ("wrist_roll_link", "wrist_yaw_link"))}
SELF_BUFFER = {f"{s}_shoulder_yaw_link": -0.01 for s in ("left", "right")}
SELF_COLLISION = False   # --self-collision


def robot_cfg():
    cfg = yaml.safe_load(open(files("curobo") / "content" / "configs" / "robot" / "unitree_g1_29dof_retarget.yml"))
    kin = cfg["kinematics"]
    kin["tool_frames"] = ["pelvis", HANDS["left"], HANDS["right"]]
    kin["lock_joints"] = dict(LEG_TUCK)
    for a, bs in SELF_IGNORE.items():
        kin["self_collision_ignore"].setdefault(a, [])
        kin["self_collision_ignore"][a] = sorted(set(kin["self_collision_ignore"][a]) | set(bs))
    kin["self_collision_buffer"].update(SELF_BUFFER)
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
        # self-collision per SELF_COLLISION (--self-collision): off reproduces the runs up to 8206c91 (the shipped
        # sphere model overlapped at rest and Isaac ran without self-collision); world collision always ON.
        # use_cuda_graph=False: graph replay crashed (CUDA illegal address) after the per-request tool-pose criteria
        # and seed shapes changed; graphs are only a speed optimisation
        common = dict(robot=cfg, scene_model=self.scene, self_collision_check=SELF_COLLISION, position_tolerance=0.01,
                      use_cuda_graph=False, optimizer_collision_activation_distance=CLEARANCE_M,
                      orientation_tolerance=3.2)
        self.ik = InverseKinematics(InverseKinematicsCfg.create(num_seeds=64, **common))
        self.ik_local = InverseKinematics(InverseKinematicsCfg.create(num_seeds=8, **common))
        self.joints = list(self.ik.joint_names)
        self.gen = torch.Generator(device="cuda").manual_seed(0)   # deterministic seed perturbations
        # geometric check independent of the optimizer: sphere model vs the ISS triangles (scene frame)
        import trimesh
        w = np.load(WORLD / "collision_world.npz")
        tri = np.concatenate([w[k].reshape(-1, 3, 3) for k in w.files])
        self.world = trimesh.proximity.ProximityQuery(
            trimesh.Trimesh(vertices=tri.reshape(-1, 3), faces=np.arange(len(tri) * 3).reshape(-1, 3), process=False))
        kin = cfg["kinematics"]
        self.sphere_link = [n for n in kin["collision_link_names"] for _ in kin["collision_spheres"].get(n, [])]

    def penetration(self, qvec):
        """Per-link max penetration (m) of the cuRobo sphere model into the ISS meshes: sphere radius minus the
        distance from its centre to the nearest triangle (unsigned: the open interior meshes have no inside)."""
        ks = self.ik.compute_kinematics(JointState.from_position(torch.as_tensor(qvec, device="cuda")[None],
                                                                joint_names=self.joints))
        sp = ks.robot_spheres.reshape(-1, 4).cpu().numpy()
        valid = sp[:, 3] > 0
        if len(sp) != len(self.sphere_link):
            raise RuntimeError(f"sphere/link map mismatch: {len(sp)} vs {len(self.sphere_link)}")
        centres = from_plan_pt(sp[valid, :3])
        _, dist, _ = self.world.on_surface(centres)
        pen = sp[valid, 3] - dist
        links = np.asarray(self.sphere_link)[valid]
        out = {}
        for ln, pv in zip(links, pen):
            if pv > 0:
                out[ln] = max(out.get(ln, 0.0), float(pv))
        return {"max_m": float(max(pen.max(), 0.0)), "links": out}

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
        # Feasibility: joint limits from cuRobo, world collision from actual geometry (sphere model vs ISS triangles,
        # no penetration). cuRobo's scene_collision term is > 0 already inside the CLEARANCE_M activation band, which
        # froze the planner whenever the measured pose was within that band; the margin still shapes the optimizer.
        pen = self.penetration(qt.reshape(-1).cpu().numpy())
        vals["penetration_m"] = pen["max_m"]
        return {"feasible": vals.get("cspace", 0.0) <= 0.0 and pen["max_m"] <= 0.0, "constraints": vals}

    def frame(self, q, feasible, goals, anchor):
        """success = feasibility (joint limits + no penetration of the ISS meshes) and anchored hand within HELD_TOL_M."""
        k = self.fk(q)
        err = {n: float(np.linalg.norm(k[n][0] - np.asarray(goals[n][0]))) for n in goals if n != "pelvis"}
        ok = feasible["feasible"] and err[HANDS[anchor]] <= HELD_TOL_M
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
        # closed-chain refinement as for the steps (Isaac holds the anchored hand at its measured pose)
        rr = {"held_hand": anchor, "free_hand": hand, "a_cur": r["target"], "held_pos": r["anchor_pos"],
              "held_quat_wxyz": r["anchor_quat_wxyz"], "pelvis_pos": r["pelvis_pos"],
              "pelvis_quat_wxyz": r["pelvis_quat_wxyz"]}
        out = (self.refine_cc(rr, {"frames": [f], "plan_success": True}) if not r.get("no_refine")
               else dict(self.closed_chain(rr, f["q"]), frames=[f], cc_refined=False))
        best, src = out, "local"
        if out["free_hand_error_cc_m"] > CC_TOL_M and not r.get("local_only"):
            # the local solve from the measured posture can stall 5-10 cm short (panel benchmark): try global
            # candidates (feasible, swing <= MAX_SWING_RAD), each with the closed-chain refinement
            act_i = [i for i, j in enumerate(self.joints) if j not in BASE]
            for qc in self.solve_candidates(goals, k=16):
                fc = self.frame(qc, self.constraints(torch.as_tensor(qc, device="cuda").view(1, 1, -1)), goals, anchor)
                if not fc["success"] or float(np.abs(qc - seed)[act_i].max()) > float(r.get("max_swing_rad", MAX_SWING_RAD)):
                    continue
                oc = self.refine_cc(rr, {"frames": [fc], "plan_success": True})
                if oc["free_hand_error_cc_m"] < best["free_hand_error_cc_m"]:
                    best, src = oc, "global"
        last = dict(best["frames"][-1], reach_error_m=best["free_hand_error_cc_m"],
                    reach_error_base_free_m=f["reach_error_m"])
        act_i = [i for i, j in enumerate(self.joints) if j not in BASE]
        dq = float(np.abs(self.qvec(last["q"], r["pelvis_pos"], r["pelvis_quat_wxyz"]) - seed)[act_i].max())
        return {"frames": [last], "cc_refined": best["cc_refined"], "reach_source": src, "max_joint_change_rad": dq,
                "suggested_duration_s": float(np.clip(SECONDS_PER_RAD * dq, 2.0, 16.0))}


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
             HANDS[held]: ToolPoseCriteria.track_position_and_orientation(xyz=[1.0] * 3, rpy=[HELD_RPY_W] * 3),
             HANDS[free]: ToolPoseCriteria.track_position(xyz=[1.0] * 3)}
        for s in (self.ik, self.ik_local):
            s.update_tool_pose_criteria(c)
        mid = (a_prev + a_cur) / 2
        pel_goal = np.array([mid[0], mid[1], CENTRE_Z + 0.15])   # aisle centre (was 0.55 m below the anchors)
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
                if fc["success"] and err <= 0.04 and dq <= MAX_SWING_RAD:     # large joint swings do not execute on PD
                    if best is None or dq < best[0]:
                        best = (dq, qc)
                elif fc["feasible"] and err <= 0.04 and (fallback is None or dq < fallback[0]):
                    fallback = (dq, qc)                          # reachable but large swing: smallest swing
            if best is not None:
                break
        partial = None
        if best is None:
            # full reach with the pelvis goal at its current position (the advancing pelvis term can make an
            # otherwise reachable anchor fail the held-hand tolerance)
            pel_now = self.fk(q_cur)["pelvis"][0]
            goals_s = {"pelvis": (pel_now, fq), HANDS[held]: held_pose, HANDS[free]: (a_cur, fq)}
            ql, okl = self.solve(goals_s, self.ik_local, q_cur)
            for qc in [ql] + self.solve_candidates(goals_s, k=16):
                fs_ = self.frame(qc, self.constraints(torch.as_tensor(qc, device="cuda").view(1, 1, -1)), goals_s, held)
                dq = float(np.abs(qc - q_cur)[act_i].max())
                if fs_["success"] and fs_["hand_goal_error_m"][HANDS[free]] <= 0.04 and dq <= MAX_SWING_RAD:
                    if best is None or dq < best[0]:
                        best, goals_f = (dq, qc), goals_s
        march = []
        if best is None:
            # no direct solution: continuation. March the free hand (and the pelvis goal) toward the anchor in
            # IK-consistent increments, each warm-started from the previous solution; the waypoints become the path.
            qm = q_cur.copy()
            for _ in range(16):
                km = self.fk(qm)
                pfm, pelm = km[HANDS[free]][0], km["pelvis"][0]
                if np.linalg.norm(a_cur - pfm) <= 0.02:
                    break
                step_ok = False
                for frac in (0.5, 0.3, 0.15):
                    goals_p = {"pelvis": (pelm + frac * (pel_goal - pelm), fq), HANDS[held]: held_pose,
                               HANDS[free]: (pfm + frac * (a_cur - pfm), fq)}
                    ql, okl = self.solve(goals_p, self.ik_local, qm)
                    fp_ = self.frame(ql, okl, goals_p, held)
                    if fp_["success"] and fp_["hand_goal_error_m"][HANDS[free]] <= 0.04 and \
                            float(np.abs(ql - qm)[act_i].max()) <= MAX_SWING_RAD:
                        qm, step_ok = ql, True
                        march.append(fp_)
                        break
                if not step_ok:
                    break
            if march:
                d_total = float(np.linalg.norm(a_cur - self.fk(q_cur)[HANDS[free]][0]))
                d_left = float(np.linalg.norm(a_cur - self.fk(qm)[HANDS[free]][0]))
                partial = round(1.0 - d_left / max(d_total, 1e-6), 3)
                best = (float(np.abs(qm - q_cur)[act_i].max()), qm)
                goals_f = {"pelvis": (pel_goal, fq), HANDS[held]: held_pose, HANDS[free]: (a_cur, fq)}
        q_final = best[1] if best is not None else q_cur                  # nothing safe: hold still
        f_final = self.frame(q_final, self.constraints(torch.as_tensor(q_final, device="cuda").view(1, 1, -1)), goals_f, held)
        act = np.array([j not in BASE for j in self.joints])
        if march:                                    # continuation waypoints are already IK-consistent
            frames, consistent = list(march[:-1]), len(march)
        else:
            # execution path: per frame, local IK (held hand fixed, free hand and pelvis interpolated) seeded by the
            # joint-space interpolation; the interpolation itself is used only where that IK fails
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
        pens = [self.penetration(np.array([f.get("base", {}).get(j, 0.0) if j in BASE else f["q"].get(j, 0.0)
                                           for j in self.joints], np.float32)) if "base" in f else None for f in frames]
        path_pen = max((p["max_m"] for p in pens if p), default=0.0)
        return {"path_max_penetration_m": path_pen,
                "path_penetrating_links": sorted({ln for p in pens if p for ln in p["links"]}),
                "path_frames_checked": sum(p is not None for p in pens),"free_hand": free, "held_hand": held, "anchor": np.asarray(a_cur).tolist(), "frames": frames,
                "free_hand_final_error_m": free_err, "final_ik_found": best is not None,
                "final_collision_free": f_final["feasible"], "path_consistent_frames": consistent, "march_waypoints": len(march),
                "path_check": "IK-consistent frames are collision-checked; interpolated fallbacks are not",
                "partial_fraction": partial,
                "max_joint_change_rad": float(np.abs(q_final - q_cur)[act_i].max()),
                "suggested_duration_s": float(np.clip(SECONDS_PER_RAD * np.abs(q_final - q_cur)[act_i].max(), 4.0, 8.0)),
                "plan_success": best is not None and partial is None and free_err <= 0.04 and path_pen <= 0.0}, q_final

    def qvec(self, qdict, pelvis_pos, pelvis_quat):
        base = base_seed(pelvis_pos, rot(pelvis_quat))
        return np.array([base[j] if j in base else qdict.get(j, 0.0) for j in self.joints], np.float32)

    def closed_chain(self, r, qdict):
        """Where the executed plan end puts the free hand: Isaac holds the held hand at its MEASURED pose (fixed joint),
        so the body is placed by the held-hand -> free-hand transform of the planned joints, not by the planned base.
        Also returns the held hand's planned orientation drift (a rotation about the held hand moves the free hand by
        about drift x arm span)."""
        k = self.fk(self.qvec(qdict, r["pelvis_pos"], r["pelvis_quat_wxyz"]))
        h, f = HANDS[r["held_hand"]], HANDS[r["free_hand"]]
        rel = rot(k[h][1]).inv().apply(k[f][0] - k[h][0])
        free = rot(r["held_quat_wxyz"]).apply(rel) + np.asarray(r["held_pos"], float)
        return {"free_hand_error_cc_m": float(np.linalg.norm(free - np.asarray(r["a_cur"], float))),
                "held_ori_drift_deg": float(np.degrees((rot(r["held_quat_wxyz"]).inv() * rot(k[h][1])).magnitude()))}

    def refine_cc(self, r, out):
        """Closed-chain refinement of a plan's last frame (in place): shift the free-hand IK goal by the closed-chain
        error (fixed-point iteration, warm-started local IK) until the free hand, placed through the measured held-hand
        pose, is within CC_TOL_M of the anchor; the refined pose is appended as the final frame only if it lowers that
        error and stays penetration-free (checked with the planned base)."""
        cc0 = self.closed_chain(r, out["frames"][-1]["q"])
        out.update(cc0)
        if cc0["free_hand_error_cc_m"] <= CC_TOL_M:
            out["cc_refined"] = False
            return out
        held, free = r["held_hand"], r["free_hand"]
        q = self.qvec(out["frames"][-1]["q"], r["pelvis_pos"], r["pelvis_quat_wxyz"])
        k = self.fk(q)
        # place the base where Isaac will put it: through the measured held-hand pose
        Tm = rot(r["held_quat_wxyz"]) * rot(k[HANDS[held]][1]).inv()
        pel = Tm.apply(k["pelvis"][0] - k[HANDS[held]][0]) + np.asarray(r["held_pos"], float)
        base = base_seed(pel, Tm * rot(k["pelvis"][1]))
        q = np.array([base[j] if j in base else qv for j, qv in zip(self.joints, q)], np.float32)
        c = {"pelvis": ToolPoseCriteria.track_position_and_orientation(xyz=[0.3] * 3, rpy=[0.1] * 3),
             HANDS[held]: ToolPoseCriteria.track_position_and_orientation(xyz=[1.0] * 3, rpy=[HELD_RPY_W] * 3),
             HANDS[free]: ToolPoseCriteria.track_position(xyz=[1.0] * 3)}
        self.ik_local.update_tool_pose_criteria(c)
        a = np.asarray(r["a_cur"], float)
        goal, best, kq = a.copy(), (cc0["free_hand_error_cc_m"], None), self.fk(q)
        for _ in range(4):
            goals = {"pelvis": (kq["pelvis"][0], kq["pelvis"][1]), HANDS[held]: (r["held_pos"], r["held_quat_wxyz"]),
                     HANDS[free]: (goal, kq[HANDS[free]][1])}
            qn, _ = self.solve(goals, self.ik_local, q)
            qd = {j: float(v) for j, v in zip(self.joints, qn) if j not in BASE}
            cc = self.closed_chain(r, qd)
            if self.penetration(qn)["max_m"] > 0.0:
                break
            if cc["free_hand_error_cc_m"] < best[0]:
                best = (cc["free_hand_error_cc_m"], qd, cc)
            if cc["free_hand_error_cc_m"] <= CC_TOL_M:
                break
            kf = self.fk(qn)
            rel = rot(kf[HANDS[held]][1]).inv().apply(kf[HANDS[free]][0] - kf[HANDS[held]][0])
            goal = goal - (rot(r["held_quat_wxyz"]).apply(rel) + np.asarray(r["held_pos"], float) - a)
            q = qn
        if best[1] is not None:
            out["frames"].append({"q": {**LEG_TUCK, **best[1]}})
            out.update(best[2])
        out["cc_refined"] = best[1] is not None
        out["plan_success"] = bool(out.get("plan_success") and out["free_hand_error_cc_m"] <= 0.04)
        out["free_hand_error_cc_before_m"] = cc0["free_hand_error_cc_m"]
        return out

    def step(self, r):
        """Closed-loop step from the measured Isaac state: {"mode": "step", "q", "pelvis_pos", "pelvis_quat_wxyz",
        "held_hand", "held_pos", "held_quat_wxyz", "free_hand", "a_prev", "a_cur", "a_next", "frames"}."""
        q = self.qvec(r["q"], r["pelvis_pos"], r["pelvis_quat_wxyz"])
        st, _ = self.plan_step(q, r["held_hand"], (r["held_pos"], r["held_quat_wxyz"]), r["free_hand"],
                               np.asarray(r["a_prev"], float), np.asarray(r["a_cur"], float),
                               np.asarray(r["a_next"], float), r["frames"])
        out = self.refine_cc(r, dict(st, frames=list(st["frames"])))
        if not out["plan_success"] and not r.get("no_fallback"):
            # no small-swing IK step: trajectory optimisation, seeded with a large re-configuration if needed
            tr = self.traj_step(dict(r, allow_large_swing=True))
            if tr["plan_success"] or tr["free_hand_error_cc_m"] < out["free_hand_error_cc_m"] - 0.02:
                return dict(tr, fallback="trajopt")
        return out

    def traj_step(self, r):
        """Closed-chain step by cuRobo trajectory optimisation from a measured state (same request as step()).

        The held hand's pose is weighted at every timestep (non-terminal = terminal), the free hand's position
        only at the end, the pelvis weakly at the end; world collision with the CLEARANCE_M margin. Returns ~40
        waypoints sub-sampled from the interpolated trajectory, its held-hand drift and penetration along the whole
        trajectory, and a duration (trajectory time x TRAJ_TIME_SCALE for joint-PD tracking).
        """
        from curobo.trajectory_optimizer import TrajectoryOptimizer, TrajectoryOptimizerCfg
        if not hasattr(self, "trajopt"):
            self.trajopt = TrajectoryOptimizer(TrajectoryOptimizerCfg.create(
                robot=robot_cfg(), scene_model=self.scene, self_collision_check=SELF_COLLISION, use_cuda_graph=False,
                optimizer_collision_activation_distance=CLEARANCE_M, num_seeds=8, position_tolerance=0.01,
                orientation_tolerance=3.2))
        held, free = r["held_hand"], r["free_hand"]
        self.trajopt.update_tool_pose_criteria({
            "pelvis": ToolPoseCriteria(terminal_pose_axes_weight_factor=[0.3, 0.3, 0.3, 0.1, 0.1, 0.1],
                                       non_terminal_pose_axes_weight_factor=[0.0] * 6),
            HANDS[held]: ToolPoseCriteria(terminal_pose_axes_weight_factor=[1.0] * 6,
                                          non_terminal_pose_axes_weight_factor=[1.0] * 6),
            HANDS[free]: ToolPoseCriteria(terminal_pose_axes_weight_factor=[5.0, 5.0, 5.0, 0.0, 0.0, 0.0],
                                          non_terminal_pose_axes_weight_factor=[0.0] * 6)})
        # free-hand terminal weight 5: at 1 it was outweighed by the held-hand term summed over the horizon
        q = self.qvec(r["q"], r["pelvis_pos"], r["pelvis_quat_wxyz"])
        a_prev, a_cur, a_next = (np.asarray(r[k], float) for k in ("a_prev", "a_cur", "a_next"))
        fq = facing_quat(a_next - a_prev)
        mid = (a_prev + a_cur) / 2
        u = np.r_[(a_next - a_prev)[:2], 0.0]
        u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-6 else np.zeros(3)
        # pelvis_shift_m: retry diversification (the executor shifts the pelvis goal forward when retries stall)
        goals = {"pelvis": (np.array([mid[0], mid[1], CENTRE_Z + 0.15]) + float(r.get("pelvis_shift_m", 0.0)) * u, fq),
                 HANDS[held]: (r["held_pos"], r["held_quat_wxyz"]), HANDS[free]: (a_cur, fq)}
        g = {k: Pose(position=T(to_plan_pt(p)), quaternion=T(wxyz(RP * rot(qq)))) for k, (p, qq) in goals.items()}
        # endpoint seed: the feasible IK candidate (no penetration, held <= HELD_TOL_M, free <= 4 cm, swing <=
        # MAX_SWING_RAD) closest to the current joints, when one exists
        act_i = [i for i, j in enumerate(self.joints) if j not in BASE]
        for s in (self.ik, self.ik_local):
            s.update_tool_pose_criteria({
                "pelvis": ToolPoseCriteria.track_position_and_orientation(xyz=[0.3] * 3, rpy=[0.1] * 3),
                HANDS[held]: ToolPoseCriteria.track_position_and_orientation(xyz=[1.0] * 3, rpy=[HELD_RPY_W] * 3),
                HANDS[free]: ToolPoseCriteria.track_position(xyz=[1.0] * 3)})
        seed, large = None, None
        for qc in self.solve_candidates(goals, k=32 if r.get("allow_large_swing") else 16):
            fc = self.frame(qc, self.constraints(torch.as_tensor(qc, device="cuda").view(1, 1, -1)), goals, held)
            dq = float(np.abs(qc - q)[act_i].max())
            if fc["success"] and fc["hand_goal_error_m"][HANDS[free]] <= 0.04:
                if dq <= MAX_SWING_RAD and (seed is None or dq < seed[0]):
                    seed = (dq, qc)
                elif dq <= LARGE_SWING_RAD and (large is None or dq < large[0]):
                    large = (dq, qc)
        large_used = seed is None and large is not None and bool(r.get("allow_large_swing"))
        if large_used:          # re-configuration: trajopt keeps the held hand and clearance along the whole path
            seed = large
        gs = None if seed is None else JointState.from_position(torch.as_tensor(seed[1], device="cuda")[None],
                                                                 joint_names=self.joints)
        self.trajopt.reset_seed()
        res = self.trajopt.solve_pose(GoalToolPose.from_poses(g, ordered_tool_frames=self.trajopt.tool_frames),
                                      current_state=JointState.from_position(torch.as_tensor(q, device="cuda")[None],
                                                                             joint_names=self.joints),
                                      goal_state=gs)
        tr = res.interpolated_trajectory
        last = int(res.interpolated_last_tstep.flatten()[0]) if res.interpolated_last_tstep is not None else None
        pos = self.active(tr)[: (last + 1) if last is not None else None]
        pick = np.unique(np.linspace(0, len(pos) - 1, min(40, len(pos))).round().astype(int))
        act = [i for i, j in enumerate(self.joints) if j not in BASE]
        drift, pen, links = 0.0, 0.0, set()
        for qv in pos[pick]:
            k = self.fk(qv)
            drift = max(drift, float(np.linalg.norm(k[HANDS[held]][0] - np.asarray(r["held_pos"]))))
            pp = self.penetration(qv)
            pen = max(pen, pp["max_m"])
            links |= set(pp["links"])
        kf = self.fk(pos[-1])
        free_err = float(np.linalg.norm(kf[HANDS[free]][0] - a_cur))
        frames = [{"q": {**LEG_TUCK, **{j: float(v) for j, v in zip(self.joints, qv) if j not in BASE}}}
                  for qv in pos[pick]]
        frames[-1].update({"pelvis_pos": kf["pelvis"][0].tolist()})
        return self.refine_cc(r, {"planner": "trajopt", "free_hand": free, "held_hand": held, "anchor": a_cur.tolist(), "frames": frames,
                "free_hand_final_error_m": free_err, "path_max_held_drift_m": drift,
                "path_max_penetration_m": pen, "path_penetrating_links": sorted(links),
                "max_joint_change_rad": float(np.abs(pos[-1] - q)[act].max()),
                "suggested_duration_s": float(np.clip(max(TRAJ_TIME_SCALE * len(pos) * self.trajopt.config.interpolation_dt,
                                                          SECONDS_PER_RAD * float(np.abs(pos[-1] - q)[act].max())),
                                                      4.0, 20.0 if large_used else 16.0)),
                "trajopt_success": bool(res.success.flatten()[0]), "goal_state_seeded": seed is not None,
                "large_swing_seed": large_used,
                "plan_success": free_err <= 0.04 and pen <= 0.0 and drift <= HELD_TOL_M})

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
        """Post-hoc geometry of a recorded run: {"mode": "check", "run": <run json>, "every": 3}. Per sampled frame
        the sphere model at the recorded pelvis pose and joints (legs at the planner tuck) vs the ISS meshes."""
        run = json.loads(Path(r["run"]).read_text(encoding="utf-8"))
        f = run["frames"]
        n, every = len(f["t"]), r.get("every", 3)
        worst, per_link, counts = 0.0, {}, {"any": 0, "gt_1cm": 0, "gt_3cm": 0, "gt_5cm": 0}
        sampled = 0
        for i in range(0, n, every):
            (pp, pq), q = f["pelvis"][i], f["q"][i]
            base = base_seed(pp, rot(pq))
            pen = self.penetration(np.array([base[j] if j in base else q.get(j, 0.0) for j in self.joints], np.float32))
            sampled += 1
            m = pen["max_m"]
            worst = max(worst, m)
            for k, th in (("any", 0.0), ("gt_1cm", 0.01), ("gt_3cm", 0.03), ("gt_5cm", 0.05)):
                counts[k] += m > th
            for ln, v in pen["links"].items():
                pl = per_link.setdefault(ln, {"frames": 0, "max_m": 0.0})
                pl["frames"] += 1
                pl["max_m"] = max(pl["max_m"], v)
        return {"frames": [], "recorded": n, "sampled": sampled, "every": every, "max_penetration_m": worst,
                "frames_penetrating": counts, "per_link": dict(sorted(per_link.items(), key=lambda x: -x[1]["frames"])),
                "note": "cuRobo G1 sphere model (legs at the planner tuck) vs ISS triangles; unsigned distance, so "
                        "'penetration' = sphere radius minus distance to the nearest surface"}


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
    ap.add_argument("--frame-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="planning-frame origin in the scene frame (default OFFSET); the base prisms reach +-10 m from it")
    ap.add_argument("--self-collision", action="store_true", help="plan with G1 self-collision (see SELF_IGNORE)")
    args = ap.parse_args()
    global OFFSET, SELF_COLLISION
    if args.frame_offset:
        OFFSET = np.asarray(args.frame_offset, float)
    SELF_COLLISION = args.self_collision
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
                    "all_success": all(f.get("success", False) for f in res["frames"])})
        out["results"].append(res)
        if "anchor_hand" in r:
            print(r["mode"], "success", res["all_success"], "feasible", [f["feasible"] for f in res["frames"]][:3],
                  "anchor err max", max(f["hand_goal_error_m"][HANDS[r["anchor_hand"]]] for f in res["frames"]), flush=True)
        else:
            print(r["mode"], "steps ok", res.get("all_steps_ok"), flush=True)
    args.out.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

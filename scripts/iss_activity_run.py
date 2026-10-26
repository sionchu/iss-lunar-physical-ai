"""Execute the full-ISS G1 hand-over-hand traverse in Isaac Sim PhysX (scenes/iss_full_v0.usda), closed loop.

C:\\isaacsim\\python.bat scripts/iss_activity_run.py --spec spec.json --out run.json [--record] [--filter-contacts]
Needs the cuRobo planner service (scripts/plan_iss_activity.py --serve PORT, spec chain.planner_port).
Physics: gate config v3 (PGS, dt 1/240, 16/4 iterations, zero gravity/damping/friction); joint PD on all DOFs with
the spec gains (whole-body PD lies outside the momentum-gate condition, see ISS_V0_MOMENTUM_GATE.md 1b). G1 =
canonical g1_29dof_rev_1_0 (scripts/iss_physics_common.py), legs held in the planner's tuck. Self-collision off.
Per step: the planner replans from the MEASURED state (held hand pose, joints, pelvis); the joints track the plan;
after a settling hold the free hand must be within CAPTURE_R of its anchor, else up to max_retries corrective
replans; capture = a scripted world fixed joint at the hand, then the previous anchor joint is disabled.
Spec control options: gains, velocity_feedforward (joint velocity targets = d(position target)/dt instead of 0, so
the damping term does not resist the planned motion), time_scale (multiplier on every move/retry duration), max_steps.
Spec hand_servo: after each move/retry (and the panel reach), a resolved-rate correction of the free hand on the
measured state (floating-base Jacobian, held hand held still in 6-D). Per step it records drive saturation: the fraction of sampled physics steps in which the PD command
|kp*e + kd*de| of any DOF reaches that DOF's max effort (URDF effort limits). Optional anchored panel reach. --filter-contacts collision-filters G1 against the ISS colliders (the S1.3/S1.4
condition); interference is then measured post hoc (plan_iss_activity.py mode "check"). Grasps are scripted
constraints: nothing here validates contact, grasping or a controller.
"""
import argparse
import hashlib
import json
import os
import math
import sys
import traceback
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("--spec", type=Path, required=True)
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--record", action="store_true", help="store link poses at 30 fps for rendering")
ap.add_argument("--filter-contacts", action="store_true",
                help="collision-group filter G1 vs ISS colliders (S1.3/S1.4 condition); interference is then measured "
                     "post hoc on the recorded poses instead of being resolved by contact")
args = ap.parse_args()
app = SimulationApp({"headless": True})

sys.path.insert(0, str(Path(__file__).resolve().parent))
import iss_physics_common as C  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_full_v0.usda"
G1_PATH = "/World/Actors/G1"
HANDS = {"left": "left_rubber_hand", "right": "right_rubber_hand"}
CAPTURE_R = 0.04
GAINS = C.POSTURE_GAINS


spec_gains = [GAINS]   # controller gains from the spec (recorded in the result)


def smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


def main():
    spec = json.loads(args.spec.read_text(encoding="utf-8"))
    spec_gains[0] = tuple(spec["chain"].get("gains", GAINS))
    cfg = json.loads(C.GATE_CONFIG.read_text(encoding="utf-8"))
    rec = {"scene": str(SCENE.relative_to(ROOT)), "scene_sha256": hashlib.sha256(SCENE.read_bytes()).hexdigest(),
           "gate_config_version": cfg.get("version"), "g1_urdf": str(C.G1_URDF.relative_to(ROOT)),
           "g1_urdf_sha256": hashlib.sha256(C.G1_URDF.read_bytes()).hexdigest(), "spec": str(args.spec),
           "spec_sha256": hashlib.sha256(args.spec.read_bytes()).hexdigest(), "capture_radius_m": CAPTURE_R,
           "posture_gains": None, "segments": [], "error": None}
    try:
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from omni.physx import get_physx_simulation_interface
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import PhysxScene, SimulationManager
        from pxr import Gf, PhysicsSchemaTools, PhysxSchema, UsdGeom, UsdPhysics

        mgr = omni.kit.app.get_app().get_extension_manager()
        mgr.set_extension_enabled_immediate("omni.scene.optimizer.core", True)
        mgr.set_extension_enabled_immediate("isaacsim.robot.schema", True)
        icfg = URDFImporterConfig()
        icfg.urdf_path = str(C.G1_URDF)
        icfg.usd_path = str(args.out.resolve().with_suffix("")) + "_g1"
        icfg.fix_base = False
        g1_usd = URDFImporter(icfg).import_urdf()

        ctx = omni.usd.get_context()
        if not ctx.open_stage(str(SCENE)):
            raise RuntimeError("cannot open scene")
        stage = ctx.get_stage()
        g1 = stage.DefinePrim(G1_PATH, "Xform")
        g1.GetReferences().AddReference(str(g1_usd))
        if args.filter_contacts:
            env = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/Env")
            env.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget("/World/Environment/ISS/Collision")
            gg = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/G1")
            gg.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(G1_PATH)
            gg.CreateFilteredGroupsRel().SetTargets([env.GetPath(), gg.GetPath()])
        rec["contacts_filtered"] = bool(args.filter_contacts)
        if spec["chain"].get("self_collision") and not os.environ.get("ISS_SELFCOL_BARE"):   # BARE: diagnosis only
            rec["added_colliders"] = C.add_g1_self_colliders(stage, g1, "/World/Environment/ISS/Collision")
        rec["posture_gains"] = list(spec_gains[0])
        for _ in range(10):
            app.update()
        phys = cfg["physics"]
        scene = UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene"))
        if scene.GetGravityMagnitudeAttr().Get() != 0.0:
            raise RuntimeError("scene gravity is not zero")
        grav = float(spec["chain"].get("gravity_m_s2", 0.0))   # comparison runs only (ground: 9.81, -z)
        scene.GetGravityMagnitudeAttr().Set(grav)
        rec["gravity_m_s2"] = grav
        stage.SetTimeCodesPerSecond(1.0 / phys["dt_s"])
        px = PhysxScene(scene.GetPrim())
        px.set_steps_per_second(round(1.0 / phys["dt_s"]))
        px.set_solver_type(phys["solver_type"])
        roots = [str(p.GetPath()) for p in stage.Traverse()
                 if p.HasAPI(UsdPhysics.ArticulationRootAPI) and str(p.GetPath()).startswith(G1_PATH + "/")]
        if len(roots) != 1:
            raise RuntimeError(f"expected one articulation root, got {roots}")
        api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(roots[0]))
        api.CreateSolverPositionIterationCountAttr(phys["position_iterations"])
        api.CreateSolverVelocityIterationCountAttr(phys["velocity_iterations"])
        self_col = bool(spec["chain"].get("self_collision", False))   # off up to 8206c91; limbs could pass through
        api.CreateEnabledSelfCollisionsAttr(self_col)
        rec["self_collision"] = self_col
        api.CreateSleepThresholdAttr(0.0)
        for p in stage.Traverse():
            if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith(G1_PATH + "/"):
                b = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
                b.CreateLinearDampingAttr(0.0)
                b.CreateAngularDampingAttr(0.0)
                b.CreateSleepThresholdAttr(0.0)
                PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.0)
        timeline = omni.timeline.get_timeline_interface()
        sim = get_physx_simulation_interface()
        dt = phys["dt_s"]
        joint_counter = [0]

        def anchor_joint(hand_path, pos, quat_wxyz):
            joint_counter[0] += 1
            j = UsdPhysics.FixedJoint.Define(stage, f"/World/Actors/G1_Anchor_{joint_counter[0]}")
            j.CreateBody1Rel().SetTargets([hand_path])
            j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in pos]))
            j.CreateLocalRot0Attr(Gf.Quatf(*[float(v) for v in quat_wxyz]))
            j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            return j

        def start_episode(state):
            timeline.stop()
            for _ in range(3):
                app.update()
            for p in list(stage.GetPrimAtPath("/World/Actors").GetChildren()):
                if p.GetName().startswith("G1_Anchor_"):
                    stage.RemovePrim(p.GetPath())
            timeline.play()
            for _ in range(12):
                app.update()
            SimulationManager.step(steps=1)
            art = Articulation(roots[0])
            if not art.is_physics_tensor_entity_valid():
                raise RuntimeError("articulation tensor invalid")
            art.set_dof_friction_properties(static_frictions=0.0, dynamic_frictions=0.0, viscous_frictions=0.0)
            names, paths = list(art.link_names), list(art.link_paths[0])
            dofs = list(art.dof_names)
            n = art.num_dofs
            q = np.array([[state["q"].get(d, 0.0) for d in dofs]], dtype=np.float32)
            art.set_world_poses(positions=[state["pelvis_pos"]], orientations=[state["pelvis_quat_wxyz"]])
            art.set_dof_positions(q)
            art.set_dof_position_targets(q)
            art.set_dof_velocities(np.zeros((1, n), dtype=np.float32))
            art.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
            g = spec_gains[0]
            art.set_dof_gains(np.full((1, n), g[0], dtype=np.float32), np.full((1, n), g[1], dtype=np.float32))
            SimulationManager.step(steps=1)
            bodies = RigidPrim(paths)
            pos, quat = (x.numpy().astype(float) for x in bodies.get_world_poses())
            a = state["anchor"]
            hi = names.index(HANDS[a["hand"]])
            j = anchor_joint(paths[hi], pos[hi], quat[hi])     # anchor at the actual hand pose after placement
            for _ in range(2):
                app.update()
            return {"art": art, "names": names, "paths": paths, "dofs": dofs, "bodies": bodies, "anchor": j,
                    "anchor_hand": a["hand"], "anchor_offset_m": float(np.linalg.norm(pos[hi] - np.asarray(a["pos"])))}

        frames = {"t": [], "poses": [], "phase": [], "q": [], "pelvis": []} if args.record else None
        clock = [0.0]

        def step(E, phase, contacts):
            SimulationManager.step(steps=1)
            clock[0] += dt
            met = E.get("met")
            if met is not None and round(clock[0] / dt) % 2 == 0:
                # loads: total external force on G1 = d(linear momentum)/dt (handhold anchor joints + structure
                # contacts); actuator work = sum |joint force * joint speed| from the projected joint forces
                lin = E["bodies"].get_velocities()[0].numpy().astype(float)
                P = (E["mass"][:, None] * lin).sum(axis=0)
                if met.get("P") is not None:
                    F = float(np.linalg.norm(P - met["P"]) / (2 * dt))
                    met["f_max"] = max(met["f_max"], F)
                    met["f"].append(F)
                    met["impulse"] += F * 2 * dt
                tau = E["art"].get_dof_projected_joint_forces().numpy()[0].astype(float)
                qd = E["art"].get_dof_velocities().numpy()[0].astype(float)
                met["energy"] += float(np.abs(tau * qd).sum()) * 2 * dt
                met["tau_max"] = max(met["tau_max"], float(np.abs(tau).max()))
                met["P"] = P
            headers, _ = sim.get_contact_report()
            for h in headers or []:
                a0, a1 = (str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1)))
                g1_side = a0 if a0.startswith(G1_PATH) else a1
                env_side = a1 if g1_side == a0 else a0
                if env_side.startswith(G1_PATH):                # G1 against itself (self-collision runs)
                    k = "|".join(sorted((a0.split("/")[-1], a1.split("/")[-1])))
                    rec.setdefault("self_contacts", {})[k] = rec.get("self_contacts", {}).get(k, 0) + 1
                    continue
                if not env_side.startswith("/World/Environment"):
                    continue
                key = f"{phase}:{g1_side.split('/')[-1]}"
                contacts[key] = contacts.get(key, 0) + 1
            if frames is not None and round(clock[0] / dt) % round(1 / (30 * dt)) == 0:
                pos, quat = (x.numpy().astype(float) for x in E["bodies"].get_world_poses())
                frames["t"].append(round(clock[0], 4))
                frames["poses"].append([pos.round(5).tolist(), quat.round(6).tolist()])
                frames["phase"].append(phase)
                pi = E["names"].index("pelvis")
                frames["pelvis"].append([pos[pi].round(5).tolist(), quat[pi].round(6).tolist()])
                frames["q"].append(dict(zip(E["dofs"], E["art"].get_dof_positions().numpy()[0].round(5).tolist())))

        def state_of(E, hand=None, target=None):
            art, names = E["art"], E["names"]
            pos, quat = (x.numpy().astype(float) for x in E["bodies"].get_world_poses())
            lin, ang = (x.numpy().astype(float) for x in E["bodies"].get_velocities())
            pi = names.index("pelvis")
            q = art.get_dof_positions().numpy()[0]
            out = {"pelvis_pos": pos[pi].tolist(), "pelvis_quat_wxyz": quat[pi].tolist(),
                   "pelvis_lin_vel": lin[pi].tolist(), "pelvis_ang_vel": ang[pi].tolist(),
                   "max_link_speed_m_s": float(np.linalg.norm(lin, axis=1).max()),
                   "q": {d: float(v) for d, v in zip(E["dofs"], q)},
                   "hands": {h: {"pos": pos[names.index(HANDS[h])].tolist(), "quat_wxyz": quat[names.index(HANDS[h])].tolist()}
                             for h in HANDS}}
            return out

        ch = spec["chain"]
        ff = bool(ch.get("velocity_feedforward", False))   # velocity targets = d(position target)/dt (else 0)
        tscale = float(ch.get("time_scale", 1.0))           # multiplier on every move/retry duration
        kp, kd = spec_gains[0]

        def track(E, path, dur_s, phase, contacts, sat):
            """Follow the smoothstep-timed piecewise-linear joint path with joint PD; count drive saturation."""
            art = E["art"]
            nst = max(1, round(tscale * dur_s / dt))
            prev = path[0]
            for k in range(nst):
                x = smooth((k + 1) / nst) * (len(path) - 1)
                i = min(int(x), len(path) - 2)
                tgt = (1 - (x - i)) * path[i] + (x - i) * path[i + 1]
                art.set_dof_position_targets(tgt[None])
                vt = (tgt - prev) / dt if ff else np.zeros_like(tgt)
                if ff:
                    art.set_dof_velocity_targets(vt[None].astype(np.float32))
                prev = tgt
                step(E, phase, contacts)
                if k % 4 == 0:
                    qn, vn = art.get_dof_positions().numpy()[0], art.get_dof_velocities().numpy()[0]
                    s = np.abs(kp * (tgt - qn) + kd * (vt - vn)) >= E["max_effort"]
                    sat["samples"] += 1
                    sat["any"] += int(s.any())
                    for j in np.flatnonzero(s):
                        sat["by_dof"][E["dofs"][j]] = sat["by_dof"].get(E["dofs"][j], 0) + 1
            if ff:
                art.set_dof_velocity_targets(np.zeros((1, len(prev)), dtype=np.float32))

        servo_on = bool(ch.get("hand_servo", False))

        def servo(E, hand, held, target, contacts, phase="servo", tol=0.75 * CAPTURE_R, iters=40):
            """Closed-loop free-hand correction (resolved rate, measured state): solve the floating-base Jacobian
            for [free-hand position = error, held-hand 6-D twist = 0] by damped least squares over the base and
            the waist/arm DOFs (legs stay at the tuck), add the DOF part to the PD targets, settle 0.1 s, repeat
            until the free hand is within tol of the target (inside the capture radius: capture right away instead of
            polishing, which drifted out again in fin_jem_0g) or progress stalls; on divergence it freezes at the
            measured joints (restoring earlier targets did not restore the hand: the floating base had moved)."""
            art, names, dofs = E["art"], E["names"], E["dofs"]
            fi, hi_ = names.index(HANDS[hand]), names.index(HANDS[held])
            cols = [6 + k for k, d in enumerate(dofs) if not any(s in d for s in ("hip", "knee", "ankle"))]
            var = list(range(6)) + cols
            qt = art.get_dof_positions().numpy()[0].copy()
            hist, best = [], 1e9
            for _ in range(iters):
                p = E["bodies"].get_world_poses()[0].numpy().astype(float)
                e = np.asarray(target, float) - p[fi]
                hist.append(float(np.linalg.norm(e)))
                best = min(best, hist[-1])
                if hist[-1] <= tol or (len(hist) > 6 and hist[-7] - hist[-1] < 0.002):
                    break
                if hist[-1] > best + 0.005:          # diverging: freeze where it is
                    q_now = art.get_dof_positions().numpy()[0]
                    for _ in range(48):
                        art.set_dof_position_targets(q_now[None].astype(np.float32))
                        step(E, phase, contacts)
                    hist.append(float(np.linalg.norm(np.asarray(target, float)
                                                     - E["bodies"].get_world_poses()[0].numpy().astype(float)[fi])))
                    break
                J = art.get_jacobian_matrices().numpy()[0].astype(float)
                A = np.vstack([J[fi, :3][:, var], J[hi_][:, var]])
                b = np.r_[(0.5 if hist[-1] > 0.06 else 0.25) * e, np.zeros(6)]   # gentler near the target
                dx = A.T @ np.linalg.solve(A @ A.T + 1e-3 * np.eye(9), b)
                dq = np.clip(dx[6:], -0.05, 0.05)
                for k, c in enumerate(cols):
                    qt[c - 6] += dq[k]
                qt = np.clip(qt, lo, hi_lim)
                for _ in range(24 if hist[-1] > 0.06 else 48):
                    art.set_dof_position_targets(qt[None].astype(np.float32))
                    step(E, phase, contacts)
            return hist

        E = start_episode(ch["start_state"])
        E["mass"] = E["bodies"].get_masses().numpy().astype(float).reshape(-1)
        rec["g1_mass_kg"] = float(E["mass"].sum())
        rec["start_anchor_offset_m"] = E["anchor_offset_m"]

        def loads(met):
            f = np.asarray(met["f"] or [0.0])
            return {"external_force_max_N": round(met["f_max"], 2), "external_force_p95_N": round(float(np.percentile(f, 95)), 2),
                    "external_force_median_N": round(float(np.median(f)), 2), "external_impulse_Ns": round(met["impulse"], 3),
                    "actuator_work_J": round(met["energy"], 3), "joint_force_max": round(met["tau_max"], 2)}
        E["max_effort"] = E["art"].get_dof_max_efforts().numpy()[0].astype(float)
        rec["dof_max_effort"] = dict(zip(E["dofs"], E["max_effort"].round(3).tolist()))
        rec["control"] = {"gains": [kp, kd], "velocity_feedforward": ff, "time_scale": tscale}
        rec["mode"] = "chain"
        dofs = E["dofs"]
        lo, hi_lim = (x.numpy()[0] for x in E["art"].get_dof_limits())
        import socket
        sock = socket.create_connection(("127.0.0.1", ch["planner_port"])).makefile("rw", encoding="utf-8")

        def ask(req):
            sock.write(json.dumps(req) + "\n")
            sock.flush()
            return json.loads(sock.readline())

        anchors = ch.get("anchors")
        n_steps = min(len(anchors) - 1, ch.get("max_steps") or len(anchors) - 1)
        for si in range(n_steps):
            s_ = state_of(E)
            held = E["anchor_hand"]
            i = si + 1
            st = ask({"mode": ch.get("planner_mode", "step"), "q": s_["q"], "pelvis_pos": s_["pelvis_pos"],
                      "pelvis_quat_wxyz": s_["pelvis_quat_wxyz"], "held_hand": held,
                      "held_pos": s_["hands"][held]["pos"], "held_quat_wxyz": s_["hands"][held]["quat_wxyz"],
                      "free_hand": "left" if held == "right" else "right", "a_prev": anchors[i - 1],
                      "a_cur": anchors[i], "a_next": anchors[min(i + 1, len(anchors) - 1)],
                      "frames": ch.get("frames", 10)})
            if st.get("error"):
                raise RuntimeError(f"planner: {st['error']}")
            st.update({"step": i, "duration_s": max(ch.get("duration_s", 2.5), st.get("suggested_duration_s", 0.0))})
            sat = {"samples": 0, "any": 0, "by_dof": {}}
            E["met"] = {"f_max": 0.0, "impulse": 0.0, "energy": 0.0, "tau_max": 0.0, "P": None, "f": []}
            contacts, res = {}, {"step": st["step"], "free_hand": st["free_hand"], "anchor": st["anchor"],
                                       "plan_success": st.get("plan_success"),
                                       "plan_free_hand_error_m": st.get("free_hand_final_error_m"),
                                       "plan_free_hand_error_cc_m": st.get("free_hand_error_cc_m"),
                                       "plan_held_ori_drift_deg": st.get("held_ori_drift_deg"),
                                       "plan_path_max_penetration_m": st.get("path_max_penetration_m"),
                                       "plan_path_penetrating_links": st.get("path_penetrating_links"),
                                       "plan_partial_fraction": st.get("partial_fraction")}
            art = E["art"]
            qs = np.array([[f["q"].get(d, 0.0) for d in dofs] for f in st["frames"]], dtype=np.float32)
            q0 = art.get_dof_positions().numpy()[0]
            track(E, np.vstack([q0[None], qs]), st["duration_s"], "move", contacts, sat)
            for k in range(round(ch.get("hold_s", 1.0) / dt)):          # let the PD converge before judging
                art.set_dof_position_targets(qs[-1][None])
                step(E, "hold", contacts)
            hand = st["free_hand"]
            hi = E["names"].index(HANDS[hand])
            target = np.asarray(st["anchor"], float)
            qe = art.get_dof_positions().numpy()[0]
            s_m = state_of(E)
            res["move_tracking"] = {"max_joint_error_rad": float(np.abs(qe - qs[-1]).max()),
                                    "worst_joint": dofs[int(np.argmax(np.abs(qe - qs[-1])))],
                                    "pelvis_pos": s_m["pelvis_pos"],
                                    "plan_pelvis_pos": st["frames"][-1].get("pelvis_pos"),
                                    "pelvis_error_m": (float(np.linalg.norm(np.asarray(s_m["pelvis_pos"]) -
                                                                            np.asarray(st["frames"][-1]["pelvis_pos"])))
                                                       if st["frames"][-1].get("pelvis_pos") else None)}
            if servo_on:
                res["servo_after_move"] = servo(E, hand, E["anchor_hand"], target, contacts)
            pos, quat = (x.numpy().astype(float) for x in E["bodies"].get_world_poses())
            d0 = d = float(np.linalg.norm(target - pos[hi]))
            retries, trace = 0, []
            while d > CAPTURE_R and retries < ch.get("max_retries", 2):
                # corrective step: replan to the same anchor from the measured state and execute it
                retries += 1
                s_ = state_of(E)
                held = E["anchor_hand"]
                fix = ask({"mode": ch.get("retry_mode", "step"), "q": s_["q"], "pelvis_pos": s_["pelvis_pos"],
                           "pelvis_quat_wxyz": s_["pelvis_quat_wxyz"], "held_hand": held,
                           "held_pos": s_["hands"][held]["pos"], "held_quat_wxyz": s_["hands"][held]["quat_wxyz"],
                           "free_hand": hand, "a_prev": anchors[st["step"] - 1], "a_cur": st["anchor"],
                           "a_next": anchors[min(st["step"] + 1, len(anchors) - 1)], "frames": ch.get("frames", 10),
                           "relax_pelvis": True})   # retries reach first: default pelvis weights (calm runs)
                if fix.get("error"):
                    break
                q0 = art.get_dof_positions().numpy()[0]
                path = np.vstack([q0[None], np.array([[f["q"].get(dn, 0.0) for dn in dofs] for f in fix["frames"]],
                                                     dtype=np.float32)])
                track(E, path, max(ch.get("retry_duration_s", 2.0), fix.get("suggested_duration_s", 0.0)), "retry",
                      contacts, sat)
                for k in range(round(ch.get("hold_s", 1.0) / dt)):
                    art.set_dof_position_targets(path[-1][None])
                    step(E, "hold", contacts)
                sv = servo(E, hand, E["anchor_hand"], target, contacts) if servo_on else None
                pos, quat = (x.numpy().astype(float) for x in E["bodies"].get_world_poses())
                d = float(np.linalg.norm(target - pos[hi]))
                trace.append({"servo": sv, "plan_free_hand_error_m": fix.get("free_hand_final_error_m"),
                              "plan_free_hand_error_cc_m": fix.get("free_hand_error_cc_m"),
                              "plan_held_ori_drift_deg": fix.get("held_ori_drift_deg"),
                              "plan_success": fix.get("plan_success"), "hand_to_anchor_m": d})
            captured = d <= CAPTURE_R
            res.update({"hand_to_anchor_after_move_m": d0, "hand_to_anchor_at_capture_m": d,
                        "corrective_replans": retries, "retry_trace": trace,
                        "drive_saturation": {"fraction_of_samples": sat["any"] / max(1, sat["samples"]),
                                             "by_dof": dict(sorted(sat["by_dof"].items(), key=lambda x: -x[1])[:5])}})
            if not captured and d > CAPTURE_R:
                res["status"] = "FAIL_NO_CAPTURE"
                res["loads"] = loads(E["met"])
                res["contacts"] = contacts
                rec["segments"].append(res)
                print(json.dumps(res), flush=True)
                break
            old = E["anchor"]
            E["anchor"] = anchor_joint(E["paths"][hi], pos[hi], quat[hi])
            old.GetPrim().CreateAttribute("physics:jointEnabled", __import__("pxr").Sdf.ValueTypeNames.Bool).Set(False)
            E["anchor_hand"] = hand
            hold = art.get_dof_positions().numpy()
            for k in range(round(ch.get("settle_s", 0.4) / dt)):
                art.set_dof_position_targets(hold)
                step(E, "settle", contacts)
            s_ = state_of(E)
            res.update({"status": "CAPTURED", "t_s": round(clock[0], 3), "pelvis_pos": s_["pelvis_pos"],
                        "pelvis_speed_m_s": float(np.linalg.norm(s_["pelvis_lin_vel"])),
                        "max_link_speed_m_s": s_["max_link_speed_m_s"], "contacts": contacts})
            res["loads"] = loads(E["met"])        # panel reach (below) is not included
            pr = None
            if ch.get("panel") and ch["panel"]["after_anchor"] == st["step"]:
                s_ = state_of(E)
                other = "left" if hand == "right" else "right"
                rp = ask({"mode": "reach", "q": s_["q"], "pelvis_pos": s_["pelvis_pos"],
                          "pelvis_quat_wxyz": s_["pelvis_quat_wxyz"], "anchor_hand": hand,
                          "anchor_pos": s_["hands"][hand]["pos"], "anchor_quat_wxyz": s_["hands"][hand]["quat_wxyz"],
                          "reach_hand": other, "target": ch["panel"]["target"]})["frames"][0]
                pr = {"q": rp["q"], "target": ch["panel"]["target"], "reach_error_m": rp["reach_error_m"]}
            if pr:
                other = "left" if hand == "right" else "right"
                ri = E["names"].index(HANDS[other])
                qa = art.get_dof_positions().numpy()[0]
                qr = np.array([pr["q"].get(dn, qa[i]) for i, dn in enumerate(dofs)], dtype=np.float32)
                dmin = 1e9
                for k in range(round(3.5 / dt)):
                    u = smooth(k * dt / 2.0)
                    art.set_dof_position_targets(((1 - u) * qa + u * qr)[None])
                    step(E, "panel_reach", contacts)
                    pp = E["bodies"].get_world_poses()[0].numpy().astype(float)[ri]
                    dmin = min(dmin, float(np.linalg.norm(pp - np.asarray(pr["target"]))))
                if servo_on:
                    sv = servo(E, other, hand, pr["target"], contacts, phase="panel_reach")
                    dmin = min(dmin, sv[-1])
                    res["panel_servo"] = sv
                res["panel_reach"] = {"hand": other, "min_distance_m": dmin, "planned_error_m": pr.get("reach_error_m")}
            rec["segments"].append(res)
            print(json.dumps({**{k: v for k, v in res.items() if k != "contacts"},
                              "contact_events": sum(contacts.values())}), flush=True)   # per-step total in the log
        rec["completed_steps"] = sum(s["status"] == "CAPTURED" for s in rec["segments"])
        rec["total_steps"] = n_steps
        rec["planning"] = "closed loop (cuRobo --serve, replanned per step from the measured state)"
        sock.close()
        timeline.stop()
        rec["sim_time_s"] = round(clock[0], 3)
        if frames is not None:
            rec["frames"] = frames
            rec["link_paths"] = E["paths"]
    except Exception as exc:
        rec["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(rec) + "\n", encoding="utf-8")
        app.close()


if __name__ == "__main__":
    main()

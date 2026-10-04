"""S1.4: PhysX run in the ISS scene - G1 IVA worksite task (drift, anchor on Handrail_B, reach to the panel).

C:\\isaacsim\\python.bat scripts/iss_physics_record.py
Physics settings come from the gate config (v3: PGS, dt 1/240, 16/4 iterations, zero gravity/damping/friction).
G1 is contact-filtered. Right-hand anchor = scripted fixed joint (world <-> hand) created when the hand reaches its
planned pose; the left hand only approaches the panel (no contact). R2 is deactivated for the physics run.
Output: outputs/iss_physics/physics_record.json (gitignored).
"""
import hashlib
import json
import sys
import traceback
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})   # must start before any pxr import

sys.path.insert(0, str(Path(__file__).resolve().parent))
import iss_physics_common as C  # noqa: E402

CONFIG = json.loads(C.GATE_CONFIG.read_text(encoding="utf-8"))


def smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


def qrot(q, v):
    w, x, y, z = q
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def main():
    C.OUT.mkdir(parents=True, exist_ok=True)
    rec = {"scene": str(C.SCENE.relative_to(C.ROOT)), "gate_config": str(C.GATE_CONFIG.relative_to(C.ROOT)),
           "gate_config_sha256": hashlib.sha256(C.GATE_CONFIG.read_bytes()).hexdigest(),
           "gate_config_version": CONFIG.get("version"), "g1_urdf": str(C.G1_URDF),
           "g1_urdf_sha256": hashlib.sha256(C.G1_URDF.read_bytes()).hexdigest(), "error": None}
    try:
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import PhysxScene, SimulationManager
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        mgr = omni.kit.app.get_app().get_extension_manager()
        mgr.set_extension_enabled_immediate("omni.scene.optimizer.core", True)
        mgr.set_extension_enabled_immediate("isaacsim.robot.schema", True)
        cfg = URDFImporterConfig()
        cfg.urdf_path = str(C.G1_URDF)
        cfg.usd_path = str(C.OUT / "g1_import")
        cfg.fix_base = CONFIG["physics"]["fix_base"]
        g1_usd = URDFImporter(cfg).import_urdf()
        rec["g1_usd"] = str(Path(g1_usd).relative_to(C.ROOT))

        ctx = omni.usd.get_context()
        if not ctx.open_stage(str(C.SCENE)):
            raise RuntimeError("cannot open scene")
        stage = ctx.get_stage()
        stage.GetPrimAtPath("/World/Actors/R2_Visual").SetActive(False)
        W = C.worksite_points(stage)
        rail_pt, panel_pt, n = (np.array(W[k]) for k in ("rail_pt", "panel_pt", "normal"))
        g1 = C.add_g1(stage, g1_usd, tuple(rail_pt + n * 0.4))   # provisional, refined after IK
        C.add_collision_groups(stage)
        for _ in range(10):
            app.update()

        phys = CONFIG["physics"]
        scene = UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene"))
        if scene.GetGravityMagnitudeAttr().Get() != 0.0:
            raise RuntimeError("scene gravity is not zero")
        stage.SetTimeCodesPerSecond(1.0 / phys["dt_s"])
        px = PhysxScene(scene.GetPrim())
        px.set_steps_per_second(round(1.0 / phys["dt_s"]))
        px.set_solver_type(phys["solver_type"])
        roots = [str(p.GetPath()) for p in stage.Traverse()
                 if p.HasAPI(UsdPhysics.ArticulationRootAPI) and str(p.GetPath()).startswith(C.G1_PATH + "/")]
        if len(roots) != 1:
            raise RuntimeError(f"expected one articulation root, got {roots}")
        api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(roots[0]))
        api.CreateSolverPositionIterationCountAttr(phys["position_iterations"])
        api.CreateSolverVelocityIterationCountAttr(phys["velocity_iterations"])
        api.CreateEnabledSelfCollisionsAttr(False)
        api.CreateSleepThresholdAttr(0.0)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                b = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                b.CreateLinearDampingAttr(0.0)
                b.CreateAngularDampingAttr(0.0)
                b.CreateSleepThresholdAttr(0.0)
        timeline = omni.timeline.get_timeline_interface()

        def start():
            timeline.play()
            for _ in range(12):
                app.update()
            SimulationManager.step(steps=1)
            art = Articulation(roots[0])
            if not art.is_physics_tensor_entity_valid():
                raise RuntimeError("articulation tensor invalid")
            art.set_dof_friction_properties(static_frictions=0.0, dynamic_frictions=0.0, viscous_frictions=0.0)
            paths = list(art.link_paths[0])
            return art, list(art.link_names), paths, RigidPrim(paths), list(art.dof_names)

        # ---- pass 1: IK with PhysX as forward kinematics (teleport + one step), hand positions in the pelvis frame ----
        cache = UsdGeom.XformCache()
        art, names, paths, bodies, dofs = start()
        pelvis_i = names.index("pelvis")
        lo, hi = (x.numpy()[0] for x in art.get_dof_limits())
        q0 = art.get_dof_positions().numpy()

        def fk(arm, qa):
            q = q0.copy()
            q[0, arm["idx"]] = qa
            art.set_dof_positions(q)
            art.set_dof_velocities(np.zeros_like(q))
            art.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
            SimulationManager.step(steps=1)
            pos, quat = (x.numpy().astype(float) for x in bodies.get_world_poses())
            return qrot(qconj(quat[pelvis_i]), pos[arm["hand_i"]] - pos[pelvis_i])

        def ik(arm, target):
            qa = q0[0, arm["idx"]].astype(float).copy()
            err = np.linalg.norm(fk(arm, qa) - target)
            step = 0.25
            while step > 0.004:
                better = False
                for j in range(len(qa)):
                    for s in (step, -step):
                        c = qa.copy()
                        c[j] = np.clip(c[j] + s, lo[arm["idx"][j]], hi[arm["idx"][j]])
                        e = np.linalg.norm(fk(arm, c) - target)
                        if e < err - 1e-5:
                            qa, err, better = c, e, True
                if not better:
                    step *= 0.5
            return qa, fk(arm, qa), float(err)

        arms = {}
        for side, a in C.ARMS.items():
            arms[side] = dict(a, idx=[dofs.index(d) for d in a["dofs"]], hand_i=names.index(a["hand"]))
        R = arms["right"]
        R["q"], R["h"], R["ik_err"] = ik(R, np.array(R["target_pelvis"]))
        timeline.stop()
        for _ in range(5):
            app.update()
        pm = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(paths[pelvis_i]))
        pq = pm.ExtractRotationQuat()
        pq = np.array([pq.GetReal(), *pq.GetImaginary()])
        want_pelvis = rail_pt - qrot(pq, R["h"])
        L = arms["left"]
        L["target_pelvis"] = qrot(qconj(pq), panel_pt - want_pelvis)
        art, names, paths, bodies, dofs = start()          # second IK pass for the left arm (fresh state)
        q0 = art.get_dof_positions().numpy()
        L["q"], L["h"], L["ik_err"] = ik(L, L["target_pelvis"])
        timeline.stop()
        for _ in range(5):
            app.update()
        delta = want_pelvis - np.array(pm.ExtractTranslation())
        op = [o for o in UsdGeom.Xformable(g1).GetOrderedXformOps() if o.GetOpName() == "xformOp:translate"][0]
        op.Set(op.Get() + Gf.Vec3d(*delta))
        rec["g1_translate"] = list(op.Get())
        rec["ik"] = {s: {"dofs": a["dofs"], "q_rad": a["q"].tolist(), "target_pelvis_m": list(map(float, a["target_pelvis"])),
                         "achieved_pelvis_m": a["h"].tolist(), "residual_m": a["ik_err"]} for s, a in arms.items()}
        rec["worksite"] = {"rail_pt": rail_pt.tolist(), "panel_pt": panel_pt.tolist()}

        # ---- feed-forward placement: the arm reach pushes the floating body back (reaction), so run short trial
        #      reaches and shift the start pose by the measured miss until the closest approach is within 1 cm ----
        def trial():
            art_, _, _, bodies_, _ = start()
            n_ = art_.num_dofs
            art_.set_dof_gains(np.full((1, n_), C.POSTURE_GAINS[0], dtype=np.float32),
                               np.full((1, n_), C.POSTURE_GAINS[1], dtype=np.float32))
            qs = art_.get_dof_positions().numpy()
            art_.set_dof_velocities(np.zeros((1, n_), dtype=np.float32))
            art_.set_velocities(linear_velocities=[list(-n * C.APPROACH_V)], angular_velocities=[[0, 0, 0]])
            best = (1e9, None)
            for k in range(round((R["t"][1] + 0.3) / phys["dt_s"])):
                u = smooth((k * phys["dt_s"] - R["t"][0]) / (R["t"][1] - R["t"][0]))
                q = qs.copy()
                q[0, R["idx"]] = qs[0, R["idx"]] + u * (R["q"] - qs[0, R["idx"]])
                art_.set_dof_position_targets(q)
                SimulationManager.step(steps=1)
                hand = bodies_.get_world_poses()[0].numpy().astype(float)[R["hand_i"]]
                d = float(np.linalg.norm(hand - rail_pt))
                if d < best[0]:
                    best = (d, hand - rail_pt)
            timeline.stop()
            for _ in range(5):
                app.update()
            return best

        rec["placement_trials_closest_m"] = []
        for _ in range(4):
            d, miss = trial()
            rec["placement_trials_closest_m"].append(d)
            if d < 0.01:
                break
            op.Set(op.Get() - Gf.Vec3d(*miss))
        rec["g1_translate"] = list(op.Get())

        # ---- pass 2: the recorded run ----
        art, names, paths, bodies, dofs = start()
        kp, kd = C.POSTURE_GAINS
        ndof = art.num_dofs
        art.set_dof_gains(np.full((1, ndof), kp, dtype=np.float32), np.full((1, ndof), kd, dtype=np.float32))
        q0 = art.get_dof_positions().numpy()
        art.set_dof_position_targets(q0)
        art.set_dof_velocities(np.zeros((1, ndof), dtype=np.float32))
        art.set_velocities(linear_velocities=[list(-n * C.APPROACH_V)], angular_velocities=[[0, 0, 0]])
        dt = phys["dt_s"]
        nsteps = round(C.DURATION / dt)
        every = round(1.0 / (C.FPS * dt))
        F = {"paths": paths, "poses": [], "t": [], "phase": [], "right_d": [], "left_d": []}
        grasp = {"status": "NOT_ACHIEVED", "t_s": None}
        post = []
        prev_rd = None
        for step in range(nsteps + 1):
            t = step * dt
            q = q0.copy()
            for a in arms.values():
                u = smooth((t - a["t"][0]) / (a["t"][1] - a["t"][0]))
                q[0, a["idx"]] = q0[0, a["idx"]] + u * (a["q"] - q0[0, a["idx"]])
            art.set_dof_position_targets(q)
            pos, quat = (x.numpy().astype(float) for x in bodies.get_world_poses())
            rh, lh = pos[R["hand_i"]], pos[L["hand_i"]]
            rd, ld = float(np.linalg.norm(rh - rail_pt)), float(np.linalg.norm(lh - panel_pt))
            if grasp["status"] != "ACHIEVED" and prev_rd is not None and prev_rd <= C.CAPTURE_R and rd > prev_rd:
                j = UsdPhysics.FixedJoint.Define(stage, "/World/Actors/G1_RightHand_Anchor")
                j.CreateBody1Rel().SetTargets([paths[R["hand_i"]]])
                hq = quat[R["hand_i"]]
                j.CreateLocalPos0Attr(Gf.Vec3f(*rh))
                j.CreateLocalRot0Attr(Gf.Quatf(*[float(v) for v in hq]))
                j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
                j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
                grasp = {"status": "ACHIEVED", "t_s": round(t, 4), "hand_pos_m": rh.tolist(), "hand_to_plan_m": rd}
            if grasp["status"] == "ACHIEVED":
                post.append(float(np.linalg.norm(rh - np.array(grasp["hand_pos_m"]))))
            prev_rd = rd
            if step % every == 0:
                if grasp["status"] != "ACHIEVED":
                    phase = "drift" if t < C.ARMS["right"]["t"][0] else "reach_rail"
                elif t < C.ARMS["left"]["t"][0]:
                    phase = "anchored"
                elif t < C.ARMS["left"]["t"][1]:
                    phase = "reach_panel"
                else:
                    phase = "at_panel"
                F["poses"].append([pos.round(6).tolist(), quat.round(7).tolist()])
                F["t"].append(round(t, 6))
                F["phase"].append(phase)
                F["right_d"].append(rd)
                F["left_d"].append(ld)
            if step < nsteps:
                SimulationManager.step(steps=1)
        timeline.stop()
        grasp["post_grasp_hand_drift_max_m"] = max(post) if post else None
        rec["grasp"] = grasp
        rec["left_hand_to_panel_point_m"] = {"min": min(F["left_d"]), "final": F["left_d"][-1]}
        rec["frames"] = F
        print(json.dumps({k: v for k, v in rec.items() if k != "frames"}), flush=True)
    except Exception as exc:
        rec["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        (C.OUT / "physics_record.json").write_text(json.dumps(rec) + "\n", encoding="utf-8")
        app.close()


if __name__ == "__main__":
    main()

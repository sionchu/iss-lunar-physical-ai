"""S1.3: PhysX run in the ISS scene - G1 posture hold (free-floating) and G1 reaching/grasping Handrail_B.

C:\\isaacsim\\python.bat scripts/iss_physics_record.py
Physics settings come from the gate config (v3: PGS, dt 1/240, 16/4 iterations, zero gravity/damping/friction).
Both G1s are contact-filtered. The grasp is a scripted fixed joint (world <-> right hand) created when the hand
comes within CAPTURE_R of the grip axis: no finger or contact physics. R2 is deactivated for the physics run.
Output: outputs/iss_physics/physics_record.json (gitignored).
"""
import hashlib
import json
import math
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


def quat_rotate(q, v):
    w, x, y, z = q
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def quat_conj(q):
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
        from momentum_measure import measure

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
        C.add_g1(stage, C.HOLD_PATH, g1_usd, C.HOLD_PLACE["translate"], C.HOLD_PLACE["rotateXYZ"])
        grip_c, grip_axis, grip_half, grip_n = C.rail_segment(stage)
        grasp_prim = C.add_g1(stage, C.GRASP_PATH, g1_usd,
                              tuple(grip_c + grip_n * 0.45), (0.0, 0.0, C.GRASP_YAW))   # provisional, refined below
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
        roots = {}
        for key, path in (("hold", C.HOLD_PATH), ("grasp", C.GRASP_PATH)):
            r = [str(p.GetPath()) for p in stage.Traverse()
                 if p.HasAPI(UsdPhysics.ArticulationRootAPI) and str(p.GetPath()).startswith(path + "/")]
            if len(r) != 1:
                raise RuntimeError(f"{key}: expected one articulation root, got {r}")
            roots[key] = r[0]
            api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(r[0]))
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
            out = {}
            for key, root in roots.items():
                art = Articulation(root)
                if not art.is_physics_tensor_entity_valid():
                    raise RuntimeError(f"{key}: articulation tensor invalid")
                art.set_dof_friction_properties(static_frictions=0.0, dynamic_frictions=0.0, viscous_frictions=0.0)
                paths = list(art.link_paths[0])
                out[key] = {"art": art, "names": list(art.link_names), "paths": paths, "bodies": RigidPrim(paths),
                            "dofs": list(art.dof_names)}
            return out

        # ---- pass 1: IK for the reach pose, using PhysX itself as forward kinematics (teleport + one step) ----
        A = start()
        g = A["grasp"]
        art, dofs = g["art"], g["dofs"]
        ri = [dofs.index(n) for n in C.REACH_DOFS]
        hand_i, pelvis_i = g["names"].index(C.HAND), g["names"].index("pelvis")
        lo, hi = (x.numpy()[0] for x in art.get_dof_limits())
        q0 = art.get_dof_positions().numpy()

        def hand_in_pelvis(qr):
            q = q0.copy()
            q[0, ri] = qr
            art.set_dof_positions(q)
            art.set_dof_velocities(np.zeros_like(q))
            art.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
            SimulationManager.step(steps=1)
            pos, quat = (x.numpy().astype(float) for x in g["bodies"].get_world_poses())
            return quat_rotate(quat_conj(quat[pelvis_i]), pos[hand_i] - pos[pelvis_i]), quat[pelvis_i]

        target = np.array(C.REACH_TARGET_PELVIS)
        qr = q0[0, ri].astype(float).copy()
        h, _ = hand_in_pelvis(qr)
        err = np.linalg.norm(h - target)
        step = 0.25
        while step > 0.004:
            improved = False
            for j in range(len(ri)):
                for s in (step, -step):
                    cand = qr.copy()
                    cand[j] = np.clip(cand[j] + s, lo[ri[j]], hi[ri[j]])
                    hc, _ = hand_in_pelvis(cand)
                    e = np.linalg.norm(hc - target)
                    if e < err - 1e-5:
                        qr, h, err, improved = cand, hc, e, True
            if not improved:
                step *= 0.5
        h_reach, _ = hand_in_pelvis(qr)
        rec["reach"] = {"dofs": C.REACH_DOFS, "q_rad": qr.tolist(), "hand_in_pelvis_m": h_reach.tolist(),
                        "ik_target_m": list(C.REACH_TARGET_PELVIS), "ik_residual_m": float(err)}
        timeline.stop()
        for _ in range(5):
            app.update()

        # ---- place G1_Grasp so that, at the reach pose, the hand sits HAND_STANDOFF aisle-side of the grip centre ----
        cache = UsdGeom.XformCache()
        pelvis_prim = stage.GetPrimAtPath(g["paths"][pelvis_i])
        pm = cache.GetLocalToWorldTransform(pelvis_prim)
        pq = pm.ExtractRotationQuat()
        pq = np.array([pq.GetReal(), *pq.GetImaginary()])
        grip = np.array(grip_c) + np.array(grip_n) * C.HAND_STANDOFF
        want_pelvis = grip - quat_rotate(pq, h_reach)
        delta = want_pelvis - np.array(pm.ExtractTranslation())
        op = [o for o in UsdGeom.Xformable(grasp_prim).GetOrderedXformOps() if o.GetOpName() == "xformOp:translate"][0]
        op.Set(op.Get() + Gf.Vec3d(*delta))
        rec["grasp_g1_translate"] = list(op.Get())

        # ---- pass 2: the recorded run ----
        A = start()
        kp, kd = C.HOLD_GAINS
        tb = CONFIG["test_b"]
        for key, a in A.items():
            n = a["art"].num_dofs
            a["art"].set_dof_gains(np.full((1, n), kp, dtype=np.float32), np.full((1, n), kd, dtype=np.float32))
            a["q0"] = a["art"].get_dof_positions().numpy()
            a["art"].set_dof_position_targets(a["q0"])
            a["art"].set_dof_velocities(np.zeros((1, n), dtype=np.float32))
        A["hold"]["art"].set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
        A["grasp"]["art"].set_velocities(linear_velocities=[list(-np.array(grip_n) * C.APPROACH_V)],
                                         angular_velocities=[[0, 0, 0]])
        hd = A["hold"]
        driven = [hd["dofs"].index(f"{s}_{j}_joint") for s in ("left", "right")
                  for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")]
        masses = hd["art"].get_link_masses().numpy()[0].astype(np.float64)
        inertias = hd["art"].get_link_inertias().numpy()[0].reshape(-1, 3, 3).astype(np.float64)
        coms = hd["art"].get_link_coms()[0].numpy()[0].astype(np.float64)
        gp_i = A["grasp"]["names"].index("pelvis")
        hp_i = hd["names"].index("pelvis")

        dt = phys["dt_s"]
        nsteps = round(C.DURATION / dt)
        every = round(1.0 / (C.FPS * dt))
        F = {"hold_paths": hd["paths"], "grasp_paths": A["grasp"]["paths"], "hold": [], "grasp": [], "t": [],
             "hand_dist": [], "grasped": [], "hold_rot_deg": []}
        L, Lref = [], []
        grasp = {"status": "NOT_ACHIEVED", "t_s": None}
        hq0 = None
        post = []
        for step in range(nsteps + 1):
            t = step * dt
            tq = hd["q0"].copy()
            tq[0, driven] = hd["q0"][0, driven] + tb["amplitude_rad"] * math.sin(2 * math.pi * tb["frequency_hz"] * t)
            hd["art"].set_dof_position_targets(tq)
            gq = A["grasp"]["q0"].copy()
            a_ = smooth((t - C.REACH_T[0]) / (C.REACH_T[1] - C.REACH_T[0]))
            gq[0, ri] = A["grasp"]["q0"][0, ri] + a_ * (qr - A["grasp"]["q0"][0, ri])
            A["grasp"]["art"].set_dof_position_targets(gq)
            m = measure(hd["bodies"], masses, inertias, coms, hd["names"])
            L.append(m["L"]); Lref.append(m["Lref"])
            gpos, gquat = (x.numpy().astype(float) for x in A["grasp"]["bodies"].get_world_poses())
            hand = gpos[hand_i]
            s_ = float(np.clip(np.dot(hand - np.array(grip_c), np.array(grip_axis)), -grip_half, grip_half))
            dist = float(np.linalg.norm(hand - (np.array(grip_c) + s_ * np.array(grip_axis))))
            if grasp["status"] != "ACHIEVED" and dist <= C.CAPTURE_R + C.HAND_STANDOFF:
                j = UsdPhysics.FixedJoint.Define(stage, "/World/Actors/G1_Grasp_Constraint")
                j.CreateBody1Rel().SetTargets([A["grasp"]["paths"][hand_i]])
                q = gquat[hand_i]
                j.CreateLocalPos0Attr(Gf.Vec3f(*hand))
                j.CreateLocalRot0Attr(Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
                j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
                j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
                grasp = {"status": "ACHIEVED", "t_s": round(t, 4), "hand_pos_m": hand.tolist(),
                         "hand_to_grip_axis_m": dist}
            if grasp["status"] == "ACHIEVED":
                post.append(float(np.linalg.norm(hand - np.array(grasp["hand_pos_m"]))))
            if step % every == 0:
                hpos, hquat = (x.numpy().astype(float) for x in hd["bodies"].get_world_poses())
                if hq0 is None:
                    hq0 = hquat[hp_i]
                F["hold"].append([hpos.round(6).tolist(), hquat.round(7).tolist()])
                F["grasp"].append([gpos.round(6).tolist(), gquat.round(7).tolist()])
                F["t"].append(round(t, 6))
                F["hand_dist"].append(dist)
                F["grasped"].append(grasp["status"] == "ACHIEVED")
                F["hold_rot_deg"].append(math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(hq0, hquat[hp_i])))))))
            if step < nsteps:
                SimulationManager.step(steps=1)
        timeline.stop()
        L = np.linalg.norm(np.asarray(L), axis=1)
        lref = max(Lref)
        ratio = float(L.max() / lref)
        rec["hold"] = {"angular_error_ratio": ratio, "angular_tolerance": tb["max_angular_momentum_fraction_of_lref"],
                       "status": "PASS" if ratio <= tb["max_angular_momentum_fraction_of_lref"] else "FAIL",
                       "drive": f"all {hd['art'].num_dofs} DOFs PD {kp}/{kd}; arms add the gate Test B sine",
                       "max_pelvis_rotation_deg": max(F["hold_rot_deg"]),
                       "running_ratio": [float(L[:i * every + 1].max() / lref) for i in range(len(F["t"]))],
                       "note": "not the gate condition (gate drives only the arms); reported, not a gate verdict"}
        grasp["post_grasp_hand_drift_max_m"] = max(post) if post else None
        grasp["min_hand_to_grip_axis_m"] = min(F["hand_dist"])
        rec["grasp"] = grasp
        rec["frames"] = F
        print(json.dumps({k: v for k, v in rec.items() if k not in ("frames", "hold")}
                         | {"hold": {k: v for k, v in rec["hold"].items() if k != "running_ratio"}}), flush=True)
    except Exception as exc:
        rec["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        (C.OUT / "physics_record.json").write_text(json.dumps(rec) + "\n", encoding="utf-8")
        app.close()


if __name__ == "__main__":
    main()

"""S1.2: run PhysX in the ISS scene (G1 arm-actuation from rest + free-floating cargo) and record poses.

C:\\isaacsim\\python.bat scripts/s12_record_physics.py
Physics settings come from the gate config (v3: PGS, dt 1/240, 16/4 iterations, zero gravity/damping).
G1 is driven exactly like gate Test B and is filtered from all contacts; cargo collides with the collision proxies.
R2 (kinematic) is deactivated for the physics run. Output: outputs/s12/physics_record.json (gitignored).
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
from s12_common import (BAGS, CARGO_PATH, DURATION, FPS, G1_PATH, G1_URDF, GATE_CONFIG, OUT, ROOT, SCENE,  # noqa: E402
                        add_cargo, add_collision_groups, add_g1)

CONFIG = json.loads(GATE_CONFIG.read_text(encoding="utf-8"))
HY, HZ, CORNER = 1.00, 0.94, 1.55   # measured aisle (scripts/build_iss_microlab.py)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rec = {"scene": str(SCENE.relative_to(ROOT)), "gate_config": str(GATE_CONFIG.relative_to(ROOT)),
           "gate_config_sha256": hashlib.sha256(GATE_CONFIG.read_bytes()).hexdigest(),
           "gate_config_version": CONFIG.get("version"), "g1_urdf": str(G1_URDF),
           "g1_urdf_sha256": hashlib.sha256(G1_URDF.read_bytes()).hexdigest(), "error": None}
    try:
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import PhysxScene, SimulationManager
        from pxr import PhysxSchema, UsdPhysics
        from momentum_measure import measure

        mgr = omni.kit.app.get_app().get_extension_manager()
        mgr.set_extension_enabled_immediate("omni.scene.optimizer.core", True)
        mgr.set_extension_enabled_immediate("isaacsim.robot.schema", True)
        cfg = URDFImporterConfig()
        cfg.urdf_path = str(G1_URDF)
        cfg.usd_path = str(OUT / "g1_import")
        cfg.fix_base = CONFIG["physics"]["fix_base"]
        g1_usd = URDFImporter(cfg).import_urdf()
        rec["g1_usd"] = str(Path(g1_usd).relative_to(ROOT))

        ctx = omni.usd.get_context()
        if not ctx.open_stage(str(SCENE)):
            raise RuntimeError("cannot open scene")
        stage = ctx.get_stage()
        stage.GetPrimAtPath("/World/Actors/R2_Visual").SetActive(False)   # kinematic actor not part of the physics run
        add_g1(stage, g1_usd)
        cargo_paths = add_cargo(stage, physics=True)
        add_collision_groups(stage)
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
                 if p.HasAPI(UsdPhysics.ArticulationRootAPI) and str(p.GetPath()).startswith(G1_PATH)]
        if len(roots) != 1:
            raise RuntimeError(f"expected one G1 articulation root, got {roots}")
        root_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(roots[0]))
        root_api.CreateSolverPositionIterationCountAttr(phys["position_iterations"])
        root_api.CreateSolverVelocityIterationCountAttr(phys["velocity_iterations"])
        root_api.CreateEnabledSelfCollisionsAttr(False)
        root_api.CreateSleepThresholdAttr(0.0)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                api.CreateLinearDampingAttr(0.0)
                api.CreateAngularDampingAttr(0.0)
                api.CreateSleepThresholdAttr(0.0)

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(12):
            app.update()
        SimulationManager.step(steps=1)   # warm-up step before setting initial conditions (gate practice)
        art = Articulation(roots[0])
        if not art.is_physics_tensor_entity_valid():
            raise RuntimeError("articulation tensor invalid")
        art.set_dof_friction_properties(static_frictions=0.0, dynamic_frictions=0.0, viscous_frictions=0.0)
        names, paths = list(art.link_names), list(art.link_paths[0])
        bodies = RigidPrim(paths)
        cargo = RigidPrim(cargo_paths)
        masses = art.get_link_masses().numpy()[0].astype(np.float64)
        inertias = art.get_link_inertias().numpy()[0].reshape(-1, 3, 3).astype(np.float64)
        coms = art.get_link_coms()[0].numpy()[0].astype(np.float64)
        dof_names = list(art.dof_names)
        tb = CONFIG["test_b"]
        driven = [dof_names.index(f"{s}_{j}_joint") for s in ("left", "right")
                  for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")]
        gains = np.zeros((1, art.num_dofs), dtype=np.float32)
        damp = np.zeros_like(gains)
        gains[0, driven], damp[0, driven] = tb["drive_stiffness"], tb["drive_damping"]
        art.set_dof_gains(gains, damp)
        targets = art.get_dof_positions().numpy()
        art.set_dof_position_targets(targets)
        art.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
        art.set_dof_velocities(np.zeros((1, art.num_dofs), dtype=np.float32))
        # cargo initial velocities are authored in USD; re-apply after the warm-up step so they start as authored
        cargo.set_velocities(np.array([b["vel"] for b in BAGS], dtype=np.float32),
                             np.array([[math.radians(v) for v in b["ang"]] for b in BAGS], dtype=np.float32))

        dt = phys["dt_s"]
        nsteps = round(DURATION / dt)
        every = round(1.0 / (FPS * dt))
        frames = {"g1_paths": paths, "cargo_paths": cargo_paths, "g1": [], "cargo": [], "t": []}
        L, P, Lref, Pref, clear = [], [], [], [], []
        for step in range(nsteps + 1):
            targets[0, driven] = tb["amplitude_rad"] * math.sin(2 * math.pi * tb["frequency_hz"] * step * dt)
            art.set_dof_position_targets(targets)
            m = measure(bodies, masses, inertias, coms, names)
            L.append(m["L"]); P.append(m["P"]); Lref.append(m["Lref"]); Pref.append(m["Pref"])
            if step % every == 0:
                gp, gq = (x.numpy().astype(float) for x in bodies.get_world_poses())
                cp, cq = (x.numpy().astype(float) for x in cargo.get_world_poses())
                frames["g1"].append([gp.round(6).tolist(), gq.round(7).tolist()])
                frames["cargo"].append([cp.round(6).tolist(), cq.round(7).tolist()])
                frames["t"].append(round(step * dt, 6))
                y, z = np.abs(gp[:, 1]), np.abs(gp[:, 2])   # link-origin clearance to the measured aisle
                clear.append(float(min((HY - y).min(), (HZ - z).min(), (CORNER - y - z).min())))
            if step < nsteps:
                SimulationManager.step(steps=1)
        timeline.stop()
        L, P = np.asarray(L), np.asarray(P)
        lref, pref = max(Lref), max(Pref)
        dl, dp = np.linalg.norm(L, axis=1), np.linalg.norm(P, axis=1)
        rec["g1_momentum"] = {
            "steps": nsteps, "Lref": lref, "Pref": pref,
            "angular_error_ratio": float(dl.max() / lref), "linear_error_ratio": float(dp.max() / pref),
            "angular_tolerance": tb["max_angular_momentum_fraction_of_lref"],
            "linear_tolerance": tb["max_linear_momentum_fraction_of_pref"],
            "per_frame_angular_ratio_running_max": [float(dl[:i * every + 1].max() / lref) for i in range(len(frames["t"]))],
            "note": "same quantities as gate Test B, measured in the ISS scene with G1 contact-filtered"}
        rec["g1_momentum"]["status"] = ("PASS" if rec["g1_momentum"]["angular_error_ratio"] <= rec["g1_momentum"]["angular_tolerance"]
                                        and rec["g1_momentum"]["linear_error_ratio"] <= rec["g1_momentum"]["linear_tolerance"] else "FAIL")
        rec["g1_min_link_origin_clearance_m"] = min(clear)
        rec["frames"] = frames
        print(json.dumps({k: v for k, v in rec.items() if k != "frames"} | {"g1_momentum": {
            k: v for k, v in rec["g1_momentum"].items() if k != "per_frame_angular_ratio_running_max"}}), flush=True)
    except Exception as exc:
        rec["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        (OUT / "physics_record.json").write_text(json.dumps(rec) + "\n", encoding="utf-8")
        app.close()


if __name__ == "__main__":
    main()

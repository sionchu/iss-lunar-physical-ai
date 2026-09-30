"""Headless Isaac Sim zero-g whole-body momentum gate for the R2 derivative."""

import json
import math
from pathlib import Path
import sys
import traceback

import numpy as np
from isaacsim import SimulationApp


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "configs/iss_v0_momentum.json"
URDF = ROOT / "outputs/r2_derived/r2c6_zero_g.urdf"
OUT = ROOT / "outputs/iss_v0_momentum_result.json"
LOG = ROOT / "outputs/iss_v0_momentum.log"
CONFIG = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def rotation(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def measure(bodies, masses, inertias, coms, com_quats, names):
    poses, quats = (x.numpy().astype(np.float64) for x in bodies.get_world_poses())
    linear, angular = (x.numpy().astype(np.float64) for x in bodies.get_velocities())
    positions = []
    velocities = []
    spins = []
    for i in range(len(masses)):
        rb = rotation(quats[i])
        offset = rb @ coms[i]
        positions.append(poses[i] + offset)
        velocities.append(linear[i] + np.cross(angular[i], offset))
        ri = rb @ rotation(com_quats[i])
        spins.append(ri @ inertias[i] @ ri.T @ angular[i])
    positions = np.asarray(positions)
    velocities = np.asarray(velocities)
    center = np.sum(masses[:, None] * positions, axis=0) / masses.sum()
    terms_p = masses[:, None] * velocities
    terms_l = np.cross(positions - center, terms_p) + spins
    return {"P": terms_p.sum(axis=0), "L": terms_l.sum(axis=0),
            "Pref": float(np.linalg.norm(terms_p, axis=1).sum()),
            "Lref": float(np.linalg.norm(terms_l, axis=1).sum()),
            "largest_P_link": names[int(np.argmax(np.linalg.norm(terms_p, axis=1)))],
            "largest_L_link": names[int(np.argmax(np.linalg.norm(terms_l, axis=1)))]}


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    result = {"config": str(CONFIG_PATH), "urdf": str(URDF), "test_a": {"status": "NOT_RUN"},
              "test_b": {"status": "NOT_RUN"}, "inertia_frame": "link body frame about link COM, rotated by link and COM orientations",
              "error": None}
    app = None
    try:
        app = SimulationApp({"headless": True})
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from pxr import PhysxSchema, UsdPhysics
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import PhysxScene, SimulationManager

        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("omni.scene.optimizer.core", True)
        manager.set_extension_enabled_immediate("isaacsim.robot.schema", True)
        cfg = URDFImporterConfig()
        cfg.urdf_path = str(URDF)
        cfg.usd_path = str(ROOT / "outputs/r2_derived/isaac_import")
        cfg.ros_package_paths = [{"name": "r2_description", "path": str(ROOT / "assets/raw/nasa-urdf-robots/r2_description")}]
        cfg.fix_base = CONFIG["physics"]["fix_base"]
        usd_path = URDFImporter(cfg).import_urdf()
        if not usd_path:
            raise RuntimeError("URDF importer returned no USD path")
        result["imported_usd"] = str(usd_path)
        context = omni.usd.get_context()
        if not context.open_stage(str(usd_path)):
            raise RuntimeError("Could not open imported USD")
        for _ in range(5):
            app.update()
        stage = context.get_stage()
        physics = CONFIG["physics"]
        scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(0.0)
        stage.SetTimeCodesPerSecond(1.0 / physics["dt_s"])
        physx_scene = PhysxScene(scene.GetPrim())
        physx_scene.set_steps_per_second(round(1.0 / physics["dt_s"]))
        physx_scene.set_solver_type(physics["solver_type"])
        roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise RuntimeError(f"Expected one articulation root, found {roots}")
        root_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(roots[0]))
        root_api.CreateSolverPositionIterationCountAttr(physics["position_iterations"])
        root_api.CreateSolverVelocityIterationCountAttr(physics["velocity_iterations"])
        root_api.CreateEnabledSelfCollisionsAttr(False)
        root_api.CreateSleepThresholdAttr(0.0)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                api.CreateLinearDampingAttr(0.0)
                api.CreateAngularDampingAttr(0.0)
                api.CreateSleepThresholdAttr(0.0)
        timeline = omni.timeline.get_timeline_interface()
        for key in ("test_a", "test_b"):
            timeline.stop()
            timeline.play()
            for _ in range(12):
                app.update()
            art = Articulation(roots[0])
            if not art.is_physics_tensor_entity_valid():
                raise RuntimeError("Articulation tensor invalid")
            art.set_dof_friction_properties(static_frictions=0.0, dynamic_frictions=0.0, viscous_frictions=0.0)
            names = list(art.link_names)
            paths = list(art.link_paths[0])
            bodies = RigidPrim(paths)
            if not bodies.is_physics_tensor_entity_valid():
                raise RuntimeError("Rigid-body tensor invalid")
            masses = art.get_link_masses().numpy()[0].astype(np.float64)
            inertias = art.get_link_inertias().numpy()[0].reshape(-1, 3, 3).astype(np.float64)
            com, com_q = art.get_link_coms()
            coms = com.numpy()[0].astype(np.float64)
            com_quats = com_q.numpy()[0].astype(np.float64)
            if len(paths) != len(masses):
                raise RuntimeError(f"Link path/mass mismatch: {len(paths)} vs {len(masses)}")
            dof_names = list(art.dof_names)
            gains = np.zeros((1, art.num_dofs), dtype=np.float32)
            targets = art.get_dof_positions().numpy()
            driven = []
            if key == "test_b":
                driven = [i for i, name in enumerate(dof_names) if any(f"{side}_armjoint{joint}_" in name for side in ("left", "right") for joint in range(4))]
                if not driven:
                    raise RuntimeError(f"No shoulder/elbow arm DOFs matched: {dof_names}")
                gains[0, driven] = CONFIG[key]["drive_stiffness"]
            damping = np.zeros_like(gains)
            if driven:
                damping[0, driven] = CONFIG[key]["drive_damping"]
            art.set_dof_gains(gains, damping)
            art.set_dof_position_targets(targets)
            if key == "test_a":
                art.set_velocities(linear_velocities=[CONFIG[key]["initial_base_linear_velocity_m_s"]],
                                   angular_velocities=[CONFIG[key]["initial_base_angular_velocity_rad_s"]])
            else:
                art.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
                art.set_dof_velocities(np.zeros((1, art.num_dofs), dtype=np.float32))
            observations = []
            nsteps = round(physics["duration_s"] / physics["dt_s"])
            for step in range(nsteps + 1):
                if key == "test_b":
                    targets[0, driven] = CONFIG[key]["amplitude_rad"] * math.sin(2 * math.pi * CONFIG[key]["frequency_hz"] * step * physics["dt_s"])
                    art.set_dof_position_targets(targets)
                data = measure(bodies, masses, inertias, coms, com_quats, names)
                observations.append(data)
                if step < nsteps:
                    SimulationManager.step(steps=1)
            p = np.asarray([x["P"] for x in observations])
            l = np.asarray([x["L"] for x in observations])
            max_p = float(np.max(np.linalg.norm(p - (p[0] if key == "test_a" else 0), axis=1)))
            max_l = float(np.max(np.linalg.norm(l - (l[0] if key == "test_a" else 0), axis=1)))
            pref = max(x["Pref"] for x in observations)
            lref = max(x["Lref"] for x in observations)
            denom_p = float(np.linalg.norm(p[0])) if key == "test_a" else pref
            denom_l = float(np.linalg.norm(l[0])) if key == "test_a" else lref
            ratio_p = max_p / denom_p if denom_p > 0 else math.inf
            ratio_l = max_l / denom_l if denom_l > 0 else math.inf
            tp = CONFIG[key]["max_relative_linear_momentum_error"] if key == "test_a" else CONFIG[key]["max_linear_momentum_fraction_of_pref"]
            tl = CONFIG[key]["max_relative_angular_momentum_error"] if key == "test_a" else CONFIG[key]["max_angular_momentum_fraction_of_lref"]
            worst = int(np.argmax(np.linalg.norm(l - (l[0] if key == "test_a" else 0), axis=1)))
            result[key] = {"status": "PASS" if ratio_p <= tp and ratio_l <= tl else "FAIL", "steps": nsteps,
                           "max_delta_P_kg_m_s": max_p, "max_delta_L_kg_m2_s": max_l,
                           "momentum_reference_P_kg_m_s": denom_p, "momentum_reference_L_kg_m2_s": denom_l,
                           "linear_error_ratio": ratio_p, "angular_error_ratio": ratio_l,
                           "linear_tolerance": tp, "angular_tolerance": tl,
                           "initial_P": p[0].tolist(), "initial_L": l[0].tolist(),
                           "final_P": p[-1].tolist(), "final_L": l[-1].tolist(),
                           "largest_link_at_worst_L": observations[worst]["largest_L_link"],
                           "driven_dofs": [dof_names[i] for i in driven]}
            print(f"{key}: {result[key]}", flush=True)
        timeline.stop()
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        OUT.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print("MOMENTUM_RESULT=" + json.dumps(result), flush=True)
        if app is not None:
            app.close()
    return 0 if all(result[x]["status"] == "PASS" for x in ("test_a", "test_b")) else 1


if __name__ == "__main__":
    sys.exit(main())

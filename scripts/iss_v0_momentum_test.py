"""Headless Isaac Sim zero-g whole-body momentum gate for the R2 derivative."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

import numpy as np
from isaacsim import SimulationApp
from momentum_measure import measure


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "configs/iss_v0_momentum.json"
URDF = ROOT / "outputs/r2_derived/r2c6_zero_g.urdf"
OUT = ROOT / "outputs/iss_v0_momentum_result.json"
LOG = ROOT / "outputs/iss_v0_momentum.log"
CONFIG = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def load_fresh_stage(app, usd_path, result):
    """Open the imported USD from disk and author the gate physics settings.

    Called once per test so that no simulation or authoring state from an
    earlier test can reach the next one. Unsaved edits on file-backed layers
    are discarded by reloading them from disk; the reloaded identifiers are
    recorded so the reset is inspectable.
    """
    import omni.timeline
    import omni.usd
    from pxr import PhysxSchema, UsdPhysics
    from isaacsim.core.simulation_manager import PhysxScene

    omni.timeline.get_timeline_interface().stop()
    context = omni.usd.get_context()
    if not context.open_stage(str(usd_path)):
        raise RuntimeError("Could not open imported USD")
    stage = context.get_stage()
    dirty = [layer.identifier for layer in stage.GetUsedLayers() if not layer.anonymous and layer.dirty]
    for layer in stage.GetUsedLayers():
        if not layer.anonymous and layer.dirty:
            layer.Reload(force=True)
    for _ in range(5):
        app.update()
    physics = CONFIG["physics"]
    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(0.0)
    stage.SetTimeCodesPerSecond(1.0 / physics["dt_s"])
    physx_scene = PhysxScene(scene.GetPrim())
    physx_scene.set_steps_per_second(round(1.0 / physics["dt_s"]))
    physx_scene.set_solver_type(physics["solver_type"])
    roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    result["articulation_roots"] = roots
    result["world_joints"] = [{"path": str(p.GetPath()),
                                "body0": [str(x) for x in UsdPhysics.Joint(p).GetBody0Rel().GetTargets()],
                                "body1": [str(x) for x in UsdPhysics.Joint(p).GetBody1Rel().GetTargets()]}
                               for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)
                               and (not UsdPhysics.Joint(p).GetBody0Rel().GetTargets()
                                    or not UsdPhysics.Joint(p).GetBody1Rel().GetTargets())]
    result["world_ref_prims"] = [{"path": str(p.GetPath()),
                                   "rigid_body": p.HasAPI(UsdPhysics.RigidBodyAPI),
                                   "mass": UsdPhysics.MassAPI(p).GetMassAttr().Get(),
                                   "diagonal_inertia": str(UsdPhysics.MassAPI(p).GetDiagonalInertiaAttr().Get())}
                                  for p in stage.Traverse() if "world_ref" in p.GetName()]
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
    return roots[0], dirty


def initial_state(art, bodies):
    """Read back the articulation state before test initial conditions are applied."""
    arrays = {"dof_positions": art.get_dof_positions().numpy()[0],
              "dof_velocities": art.get_dof_velocities().numpy()[0]}
    arrays["link_positions"], arrays["link_orientations"] = (x.numpy() for x in bodies.get_world_poses())
    arrays["link_linear_velocities"], arrays["link_angular_velocities"] = (x.numpy() for x in bodies.get_velocities())
    digest = hashlib.sha256()
    for name, value in arrays.items():
        digest.update(name.encode())
        digest.update(np.ascontiguousarray(value, dtype=np.float32).tobytes())
    return {"sha256_float32": digest.hexdigest(),
            "dof_positions": arrays["dof_positions"].tolist(),
            "dof_velocities": arrays["dof_velocities"].tolist(),
            "max_abs_dof_position": float(np.abs(arrays["dof_positions"]).max()),
            "max_abs_dof_velocity": float(np.abs(arrays["dof_velocities"]).max()),
            "max_abs_link_linear_velocity": float(np.abs(arrays["link_linear_velocities"]).max()),
            "max_abs_link_angular_velocity": float(np.abs(arrays["link_angular_velocities"]).max()),
            "root_position": arrays["link_positions"][0].tolist(),
            "root_orientation_wxyz": arrays["link_orientations"][0].tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=URDF)
    parser.add_argument("--out", type=Path, default=OUT)
    only = parser.add_mutually_exclusive_group()
    only.add_argument("--test-a-only", action="store_true")
    only.add_argument("--test-b-only", action="store_true")
    args = parser.parse_args()
    keys = ("test_a",) if args.test_a_only else ("test_b",) if args.test_b_only else ("test_a", "test_b")
    urdf = args.urdf.resolve()
    out = args.out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    result = {"config": str(CONFIG_PATH), "urdf": str(urdf), "test_a": {"status": "NOT_RUN"},
              "test_b": {"status": "NOT_RUN"}, "inertia_frame": "PhysX tensor inertia in link body frame about COM; rotated by link orientation only",
              "test_lifecycle": "fresh stage per test: imported USD reopened, dirty file layers reloaded, physics settings re-authored",
              "tests_run_in_order": list(keys), "error": None}
    app = None
    try:
        result["urdf_sha256"] = hashlib.sha256(urdf.read_bytes()).hexdigest()
        app = SimulationApp({"headless": True})
        import omni.kit.app
        import omni.timeline
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager

        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("omni.scene.optimizer.core", True)
        manager.set_extension_enabled_immediate("isaacsim.robot.schema", True)
        cfg = URDFImporterConfig()
        cfg.urdf_path = str(urdf)
        cfg.usd_path = str(out.with_suffix("")) + "_import"
        cfg.ros_package_paths = [{"name": "r2_description", "path": str(ROOT / "assets/raw/nasa-urdf-robots/r2_description")}]
        cfg.fix_base = CONFIG["physics"]["fix_base"]
        usd_path = URDFImporter(cfg).import_urdf()
        if not usd_path:
            raise RuntimeError("URDF importer returned no USD path")
        result["imported_usd"] = str(usd_path)
        physics = CONFIG["physics"]
        timeline = omni.timeline.get_timeline_interface()
        for key in keys:
            root, reloaded = load_fresh_stage(app, usd_path, result)
            timeline.play()
            for _ in range(12):
                app.update()
            # Initialize the articulation with one explicit physics step before
            # assigning the initial twist. Without this, PhysX clears tensor
            # velocities on the first measured step (verified by calibration).
            SimulationManager.step(steps=1)
            art = Articulation(root)
            if not art.is_physics_tensor_entity_valid():
                raise RuntimeError("Articulation tensor invalid")
            art.set_dof_friction_properties(static_frictions=0.0, dynamic_frictions=0.0, viscous_frictions=0.0)
            names = list(art.link_names)
            paths = list(art.link_paths[0])
            bodies = RigidPrim(paths)
            if not bodies.is_physics_tensor_entity_valid():
                raise RuntimeError("Rigid-body tensor invalid")
            state = initial_state(art, bodies)
            state["reloaded_dirty_layers"] = reloaded
            masses = art.get_link_masses().numpy()[0].astype(np.float64)
            inertias = art.get_link_inertias().numpy()[0].reshape(-1, 3, 3).astype(np.float64)
            com, _ = art.get_link_coms()
            coms = com.numpy()[0].astype(np.float64)
            if len(paths) != len(masses):
                raise RuntimeError(f"Link path/mass mismatch: {len(paths)} vs {len(masses)}")
            dof_names = list(art.dof_names)
            gains = np.zeros((1, art.num_dofs), dtype=np.float32)
            targets = art.get_dof_positions().numpy()
            driven = []
            if key == "test_b":
                driven = [i for i, name in enumerate(dof_names) if any(f"{side}_armjoint{joint}_" in name for side in ("left", "right") for joint in range(4))]
                if not driven and urdf.name == "g1_29dof.urdf":
                    expected = [f"{side}_{joint}_joint" for side in ("left", "right")
                                for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")]
                    driven = [dof_names.index(name) for name in expected if name in dof_names]
                    if len(driven) != len(expected):
                        result[key] = {"status": "NOT_RUN", "reason": "G1 shoulder/elbow DOF mapping incomplete",
                                       "matched": [dof_names[i] for i in driven]}
                        continue
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
                data = measure(bodies, masses, inertias, coms, names)
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
                           "driven_dofs": [dof_names[i] for i in driven],
                           "dof_names": dof_names, "initial_state": state}
            print(f"{key}: {result[key]}", flush=True)
        timeline.stop()
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print("MOMENTUM_RESULT=" + json.dumps(result), flush=True)
        if app is not None:
            app.close()
    return 0 if all(result[x]["status"] == "PASS" for x in keys) else 1


if __name__ == "__main__":
    sys.exit(main())

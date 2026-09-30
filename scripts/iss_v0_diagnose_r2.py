"""Diagnostic-only R2 zero-g momentum ablations; never writes gate results."""

import argparse
import json
import math
from pathlib import Path
import sys
import traceback

import numpy as np
from isaacsim import SimulationApp
from momentum_measure import measure, rotation


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "configs/iss_v0_momentum.json"
URDF = ROOT / "outputs/r2_derived/r2c6_zero_g.urdf"
OUT = ROOT / "outputs/iss_v0_momentum_result.json"
LOG = ROOT / "outputs/iss_v0_momentum.log"

def link_group(name):
    low = name.lower()
    side = "left" if "left" in low else "right" if "right" in low else "center"
    if "gripper" in low or "jaw" in low:
        return side + " leg gripper"
    if "leg" in low or any(x in low for x in ("hip", "knee", "ankle", "foot")):
        return side + " leg"
    if any(x in low for x in ("finger", "thumb", "index", "middle", "ring", "little", "palm", "hand")):
        return side + " hand"
    if any(x in low for x in ("arm", "wrist", "shoulder")):
        return side + " arm"
    return "torso/waist/head"


def link_detail(bodies, masses, inertias, coms, names):
    poses, quats = (x.numpy().astype(np.float64) for x in bodies.get_world_poses())
    linear, angular = (x.numpy().astype(np.float64) for x in bodies.get_velocities())
    positions = np.asarray([poses[i] + rotation(quats[i]) @ coms[i] for i in range(len(names))])
    center = (masses[:, None] * positions).sum(axis=0) / masses.sum()
    groups = {}
    peaks = {}
    for i, name in enumerate(names):
        r = rotation(quats[i])
        term = np.cross(positions[i] - center, masses[i] * linear[i]) + r @ inertias[i] @ r.T @ angular[i]
        group = link_group(name)
        groups[group] = groups.get(group, np.zeros(3)) + term
        peak = peaks.setdefault(group, {"L_norm": 0, "linear_m_s": 0, "angular_rad_s": 0})
        peak["linear_m_s"] = max(peak["linear_m_s"], float(np.linalg.norm(linear[i])))
        peak["angular_rad_s"] = max(peak["angular_rad_s"], float(np.linalg.norm(angular[i])))
    for group, value in groups.items():
        peaks[group]["L_norm"] = float(np.linalg.norm(value))
    return {"groups_L": {group: value.tolist() for group, value in groups.items()}, "group_peaks": peaks}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=URDF)
    parser.add_argument("--out", type=Path, default=OUT)
    parser.add_argument("--test-a-only", action="store_true")
    parser.add_argument("--test-b-only", action="store_true")
    parser.add_argument("--config", type=Path, default=CONFIG_PATH)
    parser.add_argument("--trace", action="store_true", help="write per-step and group diagnostics beside result")
    args = parser.parse_args()
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    urdf = args.urdf.resolve()
    out = args.out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    result = {"config": str(config_path), "urdf": str(urdf), "test_a": {"status": "NOT_RUN"},
              "test_b": {"status": "NOT_RUN"}, "inertia_frame": "PhysX tensor inertia in link body frame about COM; rotated by link orientation only",
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
        cfg.urdf_path = str(urdf)
        cfg.usd_path = str(out.with_suffix("")) + "_import"
        cfg.ros_package_paths = [{"name": "r2_description", "path": str(ROOT / "assets/raw/nasa-urdf-robots/r2_description")}]
        cfg.fix_base = config["physics"]["fix_base"]
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
        physics = config["physics"]
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
        timeline = omni.timeline.get_timeline_interface()
        keys = ("test_a",) if args.test_a_only else ("test_b",) if args.test_b_only else ("test_a", "test_b")
        for key in keys:
            timeline.stop()
            timeline.play()
            for _ in range(12):
                app.update()
            # Initialize the articulation with one explicit physics step before
            # assigning the initial twist. Without this, PhysX clears tensor
            # velocities on the first measured step (verified by calibration).
            SimulationManager.step(steps=1)
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
                gains[0, driven] = config[key]["drive_stiffness"]
                if config.get("diagnostic", {}).get("armature_zero_gain_kg_m2") is not None:
                    zero_gain = [i for i in range(art.num_dofs) if i not in driven]
                    armatures = art.get_dof_armatures().numpy()
                    armatures[0, zero_gain] = config["diagnostic"]["armature_zero_gain_kg_m2"]
                    art.set_dof_armatures(armatures)
                    readback = art.get_dof_armatures().numpy()[0, zero_gain]
                    if not np.allclose(readback, armatures[0, zero_gain]):
                        raise RuntimeError("Zero-gain armature readback does not match requested value")
                    result["armature_readback"] = {"joint_count": len(zero_gain),
                                                   "value_kg_m2": float(readback[0])}
            damping = np.zeros_like(gains)
            if driven:
                damping[0, driven] = config[key]["drive_damping"]
            art.set_dof_gains(gains, damping)
            art.set_dof_position_targets(targets)
            if key == "test_a":
                art.set_velocities(linear_velocities=[config[key]["initial_base_linear_velocity_m_s"]],
                                   angular_velocities=[config[key]["initial_base_angular_velocity_rad_s"]])
            else:
                art.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
                art.set_dof_velocities(np.zeros((1, art.num_dofs), dtype=np.float32))
            observations = []
            trace = []
            group_peaks = {}
            joint_peaks = {}
            lower, upper = art.get_dof_limits()
            limits = np.stack((lower.numpy()[0], upper.numpy()[0]), axis=1)
            nsteps = round(physics["duration_s"] / physics["dt_s"])
            for step in range(nsteps + 1):
                if key == "test_b":
                    targets[0, driven] = config[key]["amplitude_rad"] * math.sin(2 * math.pi * config[key]["frequency_hz"] * step * physics["dt_s"])
                    art.set_dof_position_targets(targets)
                data = measure(bodies, masses, inertias, coms, names)
                observations.append(data)
                if args.trace and key == "test_b":
                    detail = link_detail(bodies, masses, inertias, coms, names)
                    trace.append({"step": step, "time_s": step * physics["dt_s"], "P": data["P"].tolist(), "L": data["L"].tolist(),
                                  "groups_L": detail["groups_L"]})
                    for group, peak in detail["group_peaks"].items():
                        old = group_peaks.setdefault(group, {"L_norm": 0, "linear_m_s": 0, "angular_rad_s": 0})
                        for metric, value in peak.items():
                            old[metric] = max(old[metric], value)
                    q = art.get_dof_positions().numpy()[0]
                    qd = art.get_dof_velocities().numpy()[0]
                    for i, name in enumerate(dof_names):
                        old = joint_peaks.setdefault(name, {"max_abs_velocity_rad_s": 0, "limit_hits": 0, "gain_zero": i not in driven})
                        old["max_abs_velocity_rad_s"] = max(old["max_abs_velocity_rad_s"], abs(float(qd[i])))
                        if np.isfinite(limits[i]).all() and limits[i, 1] > limits[i, 0] and (q[i] <= limits[i, 0] + 1e-3 or q[i] >= limits[i, 1] - 1e-3):
                            old["limit_hits"] += 1
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
            tp = config[key]["max_relative_linear_momentum_error"] if key == "test_a" else config[key]["max_linear_momentum_fraction_of_pref"]
            tl = config[key]["max_relative_angular_momentum_error"] if key == "test_a" else config[key]["max_angular_momentum_fraction_of_lref"]
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
            if args.trace and key == "test_b":
                for i in range(1, len(trace)):
                    trace[i]["delta_L"] = (np.asarray(trace[i]["L"]) - trace[i-1]["L"]).tolist()
                trace[0]["delta_L"] = [0.0, 0.0, 0.0]
                trace_path = out.with_name(out.stem + "_trace.json")
                trace_path.write_text(json.dumps(trace) + "\n", encoding="utf-8")
                result["diagnostics"] = {"trace": str(trace_path), "group_peaks": group_peaks,
                                         "joint_peaks": joint_peaks, "mass_min_kg": float(masses.min()),
                                         "mass_max_kg": float(masses.max()),
                                         "principal_inertia_min_kg_m2": float(min(np.linalg.eigvalsh(x).min() for x in inertias)),
                                         "principal_inertia_max_kg_m2": float(max(np.linalg.eigvalsh(x).max() for x in inertias)),
                                         "link_masses": dict(zip(names, masses.tolist())),
                                         "link_principal_inertias": {name: np.linalg.eigvalsh(inertias[i]).tolist() for i, name in enumerate(names)}}
        timeline.stop()
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print("MOMENTUM_RESULT=" + json.dumps(result), flush=True)
        if app is not None:
            app.close()
    keys = ("test_a",) if args.test_a_only else ("test_b",) if args.test_b_only else ("test_a", "test_b")
    return 0 if all(result[x]["status"] == "PASS" for x in keys) else 1


if __name__ == "__main__":
    sys.exit(main())

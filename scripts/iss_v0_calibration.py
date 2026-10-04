"""Known-answer free-body and free-articulation check of the ISS momentum reader."""

import json
import math
from pathlib import Path
import traceback

import numpy as np
from isaacsim import SimulationApp

from momentum_measure import measure


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "configs/iss_v0_momentum_v3.json"   # current gate (v1 kept as history)
CONFIG = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
OUT = ROOT / "outputs/iss_v0_calibration_result.json"
DIM = (0.4, 0.2, 0.1)
MASS = 2.0


def box(stage, path, center):
    from pxr import Gf, UsdGeom, UsdPhysics

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*DIM))
    prim = cube.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateMassAttr(MASS)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0.05, 0.02, 0.0))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(math.cos(math.pi / 8), math.sin(math.pi / 8), 0, 0))
    x, y, z = DIM
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(MASS * (y*y + z*z) / 12,
                                             MASS * (x*x + z*z) / 12,
                                             MASS * (x*x + y*y) / 12))
    return prim


def run_case(app, label):
    import omni.timeline
    import omni.usd
    from pxr import Gf, PhysxSchema, UsdPhysics
    from isaacsim.core.experimental.prims import Articulation, RigidPrim
    from isaacsim.core.simulation_manager import PhysxScene, SimulationManager

    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    from pxr import UsdGeom
    root = UsdGeom.Xform.Define(stage, "/Calibration")
    physics = CONFIG["physics"]
    stage.SetTimeCodesPerSecond(1 / physics["dt_s"])
    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(0.0)
    px_scene = PhysxScene(scene.GetPrim())
    px_scene.set_steps_per_second(round(1 / physics["dt_s"]))
    px_scene.set_solver_type(physics["solver_type"])
    if label == "box":
        paths = ["/Calibration/box"]
        box(stage, paths[0], (0, 0, 0))
    else:
        paths = ["/Calibration/base", "/Calibration/arm"]
        base = box(stage, paths[0], (-0.2, 0, 0))
        box(stage, paths[1], (0.2, 0, 0))
        UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/Calibration/hinge")
        joint.CreateBody0Rel().SetTargets([paths[0]])
        joint.CreateBody1Rel().SetTargets([paths[1]])
        joint.CreateAxisAttr("Z")
        joint.CreateLocalPos0Attr(Gf.Vec3f(0.2, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(-0.2, 0, 0))
        root_api = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
        root_api.CreateSolverPositionIterationCountAttr(physics["position_iterations"])
        root_api.CreateSolverVelocityIterationCountAttr(physics["velocity_iterations"])
        root_api.CreateEnabledSelfCollisionsAttr(False)
        root_api.CreateSleepThresholdAttr(0.0)
    for path in paths:
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path))
        api.CreateLinearDampingAttr(0.0)
        api.CreateAngularDampingAttr(0.0)
        api.CreateSleepThresholdAttr(0.0)
    timeline.play()
    for _ in range(12):
        app.update()
    # Force the first explicit physics step before applying the known twist.
    # Some articulations discard a pre-step tensor velocity at initialization.
    SimulationManager.step(steps=1)
    if label == "box":
        bodies = RigidPrim(paths)
        masses = bodies.get_masses().numpy().reshape(-1).astype(np.float64)
        inertias = bodies.get_inertias().numpy().reshape(-1, 3, 3).astype(np.float64)
        com, cq = bodies.get_coms()
        coms, quats = com.numpy().astype(np.float64), cq.numpy().astype(np.float64)
        bodies.set_velocities(linear_velocities=[[0.05, 0, 0]], angular_velocities=[[0, 0, 0.1]])
    else:
        art = Articulation("/Calibration")
        art.set_dof_friction_properties(static_frictions=0.0, dynamic_frictions=0.0, viscous_frictions=0.0)
        art.set_dof_gains(np.zeros((1, art.num_dofs), dtype=np.float32),
                          np.zeros((1, art.num_dofs), dtype=np.float32))
        paths = list(art.link_paths[0])
        bodies = RigidPrim(paths)
        masses = art.get_link_masses().numpy()[0].astype(np.float64)
        inertias = art.get_link_inertias().numpy()[0].reshape(-1, 3, 3).astype(np.float64)
        com, cq = art.get_link_coms()
        coms, quats = com.numpy()[0].astype(np.float64), cq.numpy()[0].astype(np.float64)
        art.set_velocities(linear_velocities=[[0.05, 0, 0]], angular_velocities=[[0, 0, 0.1]])
    readings = []
    early = []
    for step in range(round(physics["duration_s"] / physics["dt_s"]) + 1):
        readings.append(measure(bodies, masses, inertias, coms, paths))
        if step < 3:
            lv, av = bodies.get_velocities()
            pos, _ = bodies.get_world_poses()
            early.append({"step": step, "linear": lv.numpy().tolist(), "angular": av.numpy().tolist(),
                          "positions": pos.numpy().tolist()})
        if step < round(physics["duration_s"] / physics["dt_s"]):
            SimulationManager.step(steps=1)
    timeline.stop()
    p = np.asarray([r["P"] for r in readings])
    l = np.asarray([r["L"] for r in readings])
    dp = float(np.linalg.norm(p - p[0], axis=1).max())
    dl = float(np.linalg.norm(l - l[0], axis=1).max())
    pr = dp / np.linalg.norm(p[0])
    lr = dl / np.linalg.norm(l[0])
    expected_p = [MASS * 0.05, 0, 0] if label == "box" else None
    # The two-link spin gives its COM a transverse velocity as well; the
    # conservation comparison uses its measured post-initialization state.
    result = {"status": "PASS" if pr <= CONFIG["test_a"]["max_relative_linear_momentum_error"] and lr <= CONFIG["test_a"]["max_relative_angular_momentum_error"] else "FAIL",
              "link_paths": paths, "masses_kg": masses.tolist(), "expected_initial_P": expected_p,
              "tensor_inertias": inertias.tolist(), "tensor_com_quaternions": quats.tolist(),
              "initial_P": p[0].tolist(), "initial_L": l[0].tolist(),
              "final_P": p[-1].tolist(), "final_L": l[-1].tolist(),
              "max_delta_P_kg_m_s": dp, "max_delta_L_kg_m2_s": dl,
              "linear_error_ratio": pr, "angular_error_ratio": lr,
              "first_steps": early,
              "final_positions": bodies.get_world_poses()[0].numpy().tolist(),
              "joint_targets": {"body0": [str(x) for x in joint.GetBody0Rel().GetTargets()],
                                "body1": [str(x) for x in joint.GetBody1Rel().GetTargets()]} if label != "box" else None,
              "linear_tolerance": CONFIG["test_a"]["max_relative_linear_momentum_error"],
              "angular_tolerance": CONFIG["test_a"]["max_relative_angular_momentum_error"]}
    print(f"{label}: {result}", flush=True)
    return result


def main():
    global CONFIG, CONFIG_PATH, OUT
    import argparse
    import hashlib
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG_PATH)
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    CONFIG_PATH, OUT = args.config.resolve(), args.out.resolve()
    CONFIG = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    result = {"config": str(CONFIG_PATH), "config_sha256": hashlib.sha256(CONFIG_PATH.read_bytes()).hexdigest(),
              "config_version": CONFIG.get("version", "v1"),
              "box": {"status": "NOT_RUN"}, "two_link": {"status": "NOT_RUN"}, "error": None}
    app = None
    try:
        app = SimulationApp({"headless": True})
        for label in ("box", "two_link"):
            result[label] = run_case(app, label)
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        OUT.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        if app:
            app.close()
    return 0 if all(result[x]["status"] == "PASS" for x in ("box", "two_link")) else 1


if __name__ == "__main__":
    raise SystemExit(main())

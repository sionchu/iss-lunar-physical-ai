"""Diagnostic (1b): G1 whole-body posture PD in the ISS scene, S1.3 G1_Hold condition, hold-only.

C:\\isaacsim\\python.bat scripts/iss_g1_posture_diag.py [--config CFG] [--kp 100 --kd 10] [--no-filter] --out OUT.json
Reproduces the S1.3 hold measurement (commit c760420): one free-floating G1 at the S1.3 hold placement, PD kp/kd on
all DOFs toward the start pose, both arms add the gate Test B sine about the start pose, gate physics from --config.
Ratio = max_t |L(t)| / max_t Lref (Test B definition). --no-filter removes the G1-vs-environment collision group
filter; contacts are then counted with PhysX contact reports. Diagnostic only: not a gate verdict.
"""
import argparse
import hashlib
import json
import math
import sys
import traceback
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "configs/iss_v0_momentum_v3.json")
ap.add_argument("--urdf", type=Path)
ap.add_argument("--kp", type=float, default=100.0)
ap.add_argument("--kd", type=float, default=10.0)
ap.add_argument("--no-filter", action="store_true")
ap.add_argument("--out", type=Path, required=True)
args = ap.parse_args()
app = SimulationApp({"headless": True})

sys.path.insert(0, str(Path(__file__).resolve().parent))
import iss_physics_common as C  # noqa: E402

HOLD_PATH = "/World/Actors/G1_Hold"
HOLD_PLACE = {"translate": (-1.05, 0.0, 0.10), "rotateXYZ": (0.0, 6.0, 15.0)}   # S1.3 (c760420) placement
ARM_DOFS = [f"{s}_{j}_joint" for s in ("left", "right") for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")]


def main():
    cfg_path = args.config.resolve()
    cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
    urdf = (args.urdf or C.G1_URDF).resolve()
    out = args.out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    rec = {"script": "scripts/iss_g1_posture_diag.py", "scene": str(C.SCENE.relative_to(C.ROOT)),
           "config": str(cfg_path), "config_version": cfg.get("version"),
           "config_sha256": hashlib.sha256(cfg_path.read_bytes()).hexdigest(), "urdf": str(urdf),
           "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(), "placement": HOLD_PLACE,
           "kp": args.kp, "kd": args.kd, "contact_filter": not args.no_filter, "error": None}
    try:
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from omni.physx import get_physx_simulation_interface
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import PhysxScene, SimulationManager
        from pxr import Gf, PhysicsSchemaTools, PhysxSchema, UsdGeom, UsdPhysics
        from momentum_measure import measure

        mgr = omni.kit.app.get_app().get_extension_manager()
        mgr.set_extension_enabled_immediate("omni.scene.optimizer.core", True)
        mgr.set_extension_enabled_immediate("isaacsim.robot.schema", True)
        icfg = URDFImporterConfig()
        icfg.urdf_path = str(urdf)
        icfg.usd_path = str(out.with_suffix("")) + "_import"
        icfg.fix_base = cfg["physics"]["fix_base"]
        g1_usd = URDFImporter(icfg).import_urdf()

        ctx = omni.usd.get_context()
        if not ctx.open_stage(str(C.SCENE)):
            raise RuntimeError("cannot open scene")
        stage = ctx.get_stage()
        stage.GetPrimAtPath("/World/Actors/R2_Visual").SetActive(False)
        prim = stage.DefinePrim(HOLD_PATH, "Xform")
        prim.GetReferences().AddReference(str(g1_usd))
        x = UsdGeom.Xformable(prim)
        x.ClearXformOpOrder()
        x.AddTranslateOp().Set(Gf.Vec3d(*HOLD_PLACE["translate"]))
        x.AddRotateXYZOp().Set(Gf.Vec3f(*HOLD_PLACE["rotateXYZ"]))
        if not args.no_filter:
            env = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/Env")
            env.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget("/World/Environment/ISS/Collision")
            g = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/G1_Hold")
            g.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(HOLD_PATH)
            g.CreateFilteredGroupsRel().SetTargets([env.GetPath(), g.GetPath()])
        for _ in range(10):
            app.update()

        phys = cfg["physics"]
        scene = UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene"))
        if scene.GetGravityMagnitudeAttr().Get() != 0.0:
            raise RuntimeError("scene gravity is not zero")
        stage.SetTimeCodesPerSecond(1.0 / phys["dt_s"])
        px = PhysxScene(scene.GetPrim())
        px.set_steps_per_second(round(1.0 / phys["dt_s"]))
        px.set_solver_type(phys["solver_type"])
        roots = [str(p.GetPath()) for p in stage.Traverse()
                 if p.HasAPI(UsdPhysics.ArticulationRootAPI) and str(p.GetPath()).startswith(HOLD_PATH + "/")]
        if len(roots) != 1:
            raise RuntimeError(f"expected one articulation root, got {roots}")
        api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(roots[0]))
        api.CreateSolverPositionIterationCountAttr(phys["position_iterations"])
        api.CreateSolverVelocityIterationCountAttr(phys["velocity_iterations"])
        api.CreateEnabledSelfCollisionsAttr(False)
        api.CreateSleepThresholdAttr(0.0)
        for p in stage.Traverse():
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                b = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
                b.CreateLinearDampingAttr(0.0)
                b.CreateAngularDampingAttr(0.0)
                b.CreateSleepThresholdAttr(0.0)
                if args.no_filter and str(p.GetPath()).startswith(HOLD_PATH + "/"):
                    PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.0)

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(12):
            app.update()
        SimulationManager.step(steps=1)
        art = Articulation(roots[0])
        if not art.is_physics_tensor_entity_valid():
            raise RuntimeError("articulation tensor invalid")
        art.set_dof_friction_properties(static_frictions=0.0, dynamic_frictions=0.0, viscous_frictions=0.0)
        names, paths = list(art.link_names), list(art.link_paths[0])
        bodies = RigidPrim(paths)
        dofs = list(art.dof_names)
        driven = [dofs.index(n) for n in ARM_DOFS]
        n = art.num_dofs
        art.set_dof_gains(np.full((1, n), args.kp, dtype=np.float32), np.full((1, n), args.kd, dtype=np.float32))
        q0 = art.get_dof_positions().numpy()
        art.set_dof_position_targets(q0)
        art.set_dof_velocities(np.zeros((1, n), dtype=np.float32))
        art.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
        lo, hi = (v.numpy()[0] for v in art.get_dof_limits())
        rec["start_q_max_abs_rad"] = float(np.abs(q0).max())
        rec["start_q_outside_limits"] = [dofs[i] for i in range(n) if not lo[i] - 1e-6 <= q0[0, i] <= hi[i] + 1e-6]
        masses = art.get_link_masses().numpy()[0].astype(np.float64)
        inertias = art.get_link_inertias().numpy()[0].reshape(-1, 3, 3).astype(np.float64)
        coms = art.get_link_coms()[0].numpy()[0].astype(np.float64)
        pelvis = names.index("pelvis")
        tb = cfg["test_b"]
        dt = phys["dt_s"]
        nsteps = round(phys["duration_s"] / dt)
        sim = get_physx_simulation_interface()
        P, L, Pref, Lref, rot = [], [], [], [], []
        contacts = {"pairs_total": 0, "steps_with_contact": 0, "first_contact_t_s": None, "actors": set()}
        hq0 = None
        for step in range(nsteps + 1):
            t = step * dt
            tq = q0.copy()
            tq[0, driven] = q0[0, driven] + tb["amplitude_rad"] * math.sin(2 * math.pi * tb["frequency_hz"] * t)
            art.set_dof_position_targets(tq)
            m = measure(bodies, masses, inertias, coms, names)
            P.append(m["P"]); L.append(m["L"]); Pref.append(m["Pref"]); Lref.append(m["Lref"])
            quat = bodies.get_world_poses()[1].numpy().astype(float)[pelvis]
            hq0 = quat if hq0 is None else hq0
            rot.append(math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(hq0, quat)))))))
            if step < nsteps:
                SimulationManager.step(steps=1)
                if args.no_filter:
                    headers, _ = sim.get_contact_report()
                    if headers:
                        contacts["pairs_total"] += len(headers)
                        contacts["steps_with_contact"] += 1
                        contacts["first_contact_t_s"] = contacts["first_contact_t_s"] or round(t + dt, 5)
                        for h in headers:
                            contacts["actors"].add(str(PhysicsSchemaTools.intToSdfPath(h.actor0)))
                            contacts["actors"].add(str(PhysicsSchemaTools.intToSdfPath(h.actor1)))
        timeline.stop()
        Ln, Pn = np.linalg.norm(np.asarray(L), axis=1), np.linalg.norm(np.asarray(P), axis=1)
        ratio_l, ratio_p = float(Ln.max() / max(Lref)), float(Pn.max() / max(Pref))
        contacts["actors"] = sorted(contacts["actors"])[:40]
        rec.update({"steps": nsteps, "angular_error_ratio": ratio_l, "linear_error_ratio": ratio_p,
                    "angular_tolerance": tb["max_angular_momentum_fraction_of_lref"],
                    "linear_tolerance": tb["max_linear_momentum_fraction_of_pref"],
                    "status_vs_test_b_limits": "PASS" if ratio_l <= tb["max_angular_momentum_fraction_of_lref"]
                    and ratio_p <= tb["max_linear_momentum_fraction_of_pref"] else "FAIL",
                    "Lref_kg_m2_s": float(max(Lref)), "Pref_kg_m_s": float(max(Pref)),
                    "worst_L_t_s": float(np.argmax(Ln) * dt), "final_L": np.asarray(L[-1]).tolist(),
                    "final_P": np.asarray(P[-1]).tolist(),
                    "max_pelvis_rotation_deg": max(rot),
                    "contacts": contacts if args.no_filter else "filtered (no contact report)",
                    "note": "diagnostic: whole-body PD is not the gate condition; no gate verdict"})
        print("POSTURE_RESULT=" + json.dumps({k: v for k, v in rec.items() if k != "contacts"}), flush=True)
    except Exception as exc:
        rec["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        out.write_text(json.dumps(rec, indent=1, default=str) + "\n", encoding="utf-8")
        app.close()


if __name__ == "__main__":
    main()

"""Short microgravity demos in the full ISS scene (Isaac Sim PhysX, gate v3 physics, contacts ON), recorded for rendering.

C:\\isaacsim\\python.bat scripts/iss_space_demos.py --demo handoff|pushoff [--from-run fin_main_0g_v2 --step 9]
Start state: a captured step of a completed traverse run (joints, pelvis pose, held-hand pose from its record); the
held hand is re-anchored by a scripted fixed joint, as in iss_activity_run.py.

handoff  An Astrobee free-flyer (NASA astrobee_freeflyer meshes; mass/inertia from nasa/astrobee model.urdf.xacro;
         collision box 0.32 m) carries a tool bar, flies to a release point in front of G1 and lets it go with a
         small scripted push (0.06 m/s, 0.4 rad/s). Its propulsion is approximated by a PD force/torque on its body,
         force-limited to ASTROBEE_FORCE_N (assumed; not a sourced Astrobee value). G1 holds its handhold and tracks
         the drifting tool with its free hand by resolved rate on the measured state (same scheme as the traverse
         hand servo); capture = scripted fixed joint once the hand is within CAPTURE_R of the tool bar's axis.
pushoff  G1 pushes its pelvis along the aisle (+x) at PUSH_SPEED for PUSH_S seconds (resolved rate, held hand still)
         and releases the handhold; it then drifts and
         spins with joint PD holding its posture. Total linear and angular momentum of G1 (about its centre of mass,
         from link masses, COMs, inertias and velocities) is recorded to show what is conserved after release until
         the first contact with the structure.
Output: outputs/iss_demos/<demo>_run.json (frames for render_iss_activity.py --run, metrics). Tool and the Astrobee
force model are PLACEHOLDERs; nothing here validates grasping, Astrobee propulsion or a controller.
"""
import argparse
import hashlib
import json
import sys
import traceback
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("--demo", choices=["handoff", "pushoff"], required=True)
ap.add_argument("--from-run", default="fin_main_0g_v2")
ap.add_argument("--step", type=int, default=9, help="start from the state just after this step's capture")
ap.add_argument("--seconds", type=float, default=40.0)
args = ap.parse_args()
app = SimulationApp({"headless": True})

sys.path.insert(0, str(Path(__file__).resolve().parent))
import iss_physics_common as C  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_full_v0.usda"
OUT = ROOT / "outputs" / "iss_demos"
MEDIA = ROOT / "assets" / "cache" / "iss_media"
G1_PATH = "/World/Actors/G1"
HANDS = {"left": "left_rubber_hand", "right": "right_rubber_hand"}
CAPTURE_R = 0.04
GAINS = (300.0, 30.0)
ASTROBEE = {"mass_kg": 9.0877, "inertia": (0.1454, 0.1366, 0.1594), "box_m": 0.32,
            "source": "nasa/astrobee description/description/urdf/model.urdf.xacro (body inertial)"}
ASTROBEE_FORCE_N = 0.5          # assumed force limit of the PD propulsion stand-in (not a sourced Astrobee value)
TOOL = {"size_m": (0.30, 0.045, 0.045), "mass_kg": 1.0}
PUSH_S = 1.2
PUSH_SPEED = 0.12          # commanded pelvis speed along the aisle during the push (m/s)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    src = json.loads((ROOT / "outputs" / "iss_activity" / f"{args.from_run}_run.json").read_text(encoding="utf-8"))
    seg = next(s for s in src["segments"] if s["step"] == args.step and s["status"] == "CAPTURED")
    fr = src["frames"]
    i0 = int(np.argmin(np.abs(np.asarray(fr["t"]) - seg["t_s"])))
    names0 = [p.split("/")[-1] for p in src["link_paths"]]
    held = seg["free_hand"]                         # the hand that captured this step holds on
    free = "left" if held == "right" else "right"
    cfg = json.loads(C.GATE_CONFIG.read_text(encoding="utf-8"))
    rec = {"demo": args.demo, "scene": str(SCENE.relative_to(ROOT)), "from_run": args.from_run, "from_step": args.step,
           "start_t_s": fr["t"][i0], "held_hand": held, "free_hand": free, "gate_config_version": cfg.get("version"),
           "g1_urdf": str(C.G1_URDF.relative_to(ROOT)), "capture_radius_m": CAPTURE_R, "gains": list(GAINS),
           "contacts_filtered": False, "gravity_m_s2": 0.0, "segments": [], "error": None}
    try:
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from omni.physx import get_physx_simulation_interface
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.core.simulation_manager import PhysxScene, SimulationManager
        from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        mgr = omni.kit.app.get_app().get_extension_manager()
        mgr.set_extension_enabled_immediate("omni.scene.optimizer.core", True)
        mgr.set_extension_enabled_immediate("isaacsim.robot.schema", True)
        icfg = URDFImporterConfig()
        icfg.urdf_path = str(C.G1_URDF)
        icfg.usd_path = str(OUT / f"{args.demo}_run_g1")
        icfg.fix_base = False
        g1_usd = URDFImporter(icfg).import_urdf()
        ctx = omni.usd.get_context()
        if not ctx.open_stage(str(SCENE)):
            raise RuntimeError("cannot open scene")
        stage = ctx.get_stage()
        stage.DefinePrim(G1_PATH, "Xform").GetReferences().AddReference(str(g1_usd))

        extras = {}            # rigid bodies replayed by the renderer: name -> {path, kind, ...}
        tool = None
        if args.demo == "handoff":
            tool = UsdGeom.Cube.Define(stage, "/World/Actors/Tool")
            tool.CreateSizeAttr(1.0)
            tool.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.45, 0.1)])
            UsdGeom.Xformable(tool).AddTranslateOp()
            UsdGeom.Xformable(tool).AddOrientOp()
            UsdGeom.Xformable(tool).AddScaleOp().Set(Gf.Vec3f(*TOOL["size_m"]))
            UsdPhysics.CollisionAPI.Apply(tool.GetPrim())
            UsdPhysics.RigidBodyAPI.Apply(tool.GetPrim())
            UsdPhysics.MassAPI.Apply(tool.GetPrim()).CreateMassAttr(TOOL["mass_kg"])
            tool.GetPrim().SetCustomDataByKey("provenance", "PLACEHOLDER tool bar (box), 1.0 kg")
            extras["tool"] = {"path": "/World/Actors/Tool", "kind": "box", "size_m": list(TOOL["size_m"]),
                              "color": [0.95, 0.45, 0.1]}
            ab = UsdGeom.Xform.Define(stage, "/World/Actors/Astrobee")
            UsdGeom.Xformable(ab).AddTranslateOp()
            UsdGeom.Xformable(ab).AddOrientOp()
            vis = []
            for name, rot in (("astrobee_body", None), ("astrobee_pmc", None), ("astrobee_pmc_bumper", None),
                              ("astrobee_pmc_skin_bumble", None), ("astrobee_pmc", "stbd"),
                              ("astrobee_pmc_bumper", "stbd"), ("astrobee_pmc_skin_bumble", "stbd")):
                v = stage.DefinePrim(f"/World/Actors/Astrobee/Visual/{name}{'_stbd' if rot else ''}", "Xform")
                v.GetReferences().AddReference(str(MEDIA / f"{name}.usd"))
                if rot:                              # URDF starboard PMC: rpy (0, pi, pi) = rotate 180 deg about x
                    UsdGeom.Xformable(v).AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))
                vis.append({"usd": str((MEDIA / f"{name}.usd").relative_to(ROOT)), "rotate_x_deg": 180.0 if rot else 0.0})
            col = UsdGeom.Cube.Define(stage, "/World/Actors/Astrobee/Collision")
            col.CreateSizeAttr(ASTROBEE["box_m"])
            UsdGeom.Imageable(col).CreatePurposeAttr("guide")
            UsdPhysics.CollisionAPI.Apply(col.GetPrim())
            UsdPhysics.RigidBodyAPI.Apply(ab.GetPrim())
            m = UsdPhysics.MassAPI.Apply(ab.GetPrim())
            m.CreateMassAttr(ASTROBEE["mass_kg"])
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*ASTROBEE["inertia"]))
            m.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
            extras["astrobee"] = {"path": "/World/Actors/Astrobee", "kind": "usd_parts", "parts": vis}
            rec["astrobee"] = {**ASTROBEE, "force_limit_N": ASTROBEE_FORCE_N, "propulsion": "PD force/torque stand-in"}
            rec["tool"] = TOOL
        rec["extras"] = extras
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
                 if p.HasAPI(UsdPhysics.ArticulationRootAPI) and str(p.GetPath()).startswith(G1_PATH + "/")]
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
                if str(p.GetPath()).startswith("/World/Actors"):
                    PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.0)
        timeline = omni.timeline.get_timeline_interface()
        sim = get_physx_simulation_interface()
        dt = phys["dt_s"]
        timeline.play()
        for _ in range(12):
            app.update()
        SimulationManager.step(steps=1)
        art = Articulation(roots[0])
        art.set_dof_friction_properties(static_frictions=0.0, dynamic_frictions=0.0, viscous_frictions=0.0)
        names, paths, dofs = list(art.link_names), list(art.link_paths[0]), list(art.dof_names)
        n = art.num_dofs
        q0 = np.array([[fr["q"][i0].get(d, 0.0) for d in dofs]], dtype=np.float32)
        pel = fr["pelvis"][i0]
        art.set_world_poses(positions=[pel[0]], orientations=[pel[1]])
        art.set_dof_positions(q0)
        art.set_dof_position_targets(q0)
        art.set_dof_velocities(np.zeros((1, n), dtype=np.float32))
        art.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
        art.set_dof_gains(np.full((1, n), GAINS[0], dtype=np.float32), np.full((1, n), GAINS[1], dtype=np.float32))
        SimulationManager.step(steps=1)
        bodies = RigidPrim(paths)
        mass = bodies.get_masses().numpy().astype(float).reshape(-1)
        com_local = bodies.get_coms()[0].numpy().astype(float).reshape(-1, 3)
        inertia = bodies.get_inertias().numpy().astype(float).reshape(-1, 3, 3)
        rec["g1_mass_kg"] = float(mass.sum())
        jc = [0]

        def fixed(body_path, pos, quat, body0=None, pos1=(0, 0, 0), quat1=(1, 0, 0, 0)):
            jc[0] += 1
            j = UsdPhysics.FixedJoint.Define(stage, f"/World/Actors/DemoJoint_{jc[0]}")
            if body0:
                j.CreateBody0Rel().SetTargets([body0])
            j.CreateBody1Rel().SetTargets([body_path])
            j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in pos]))
            j.CreateLocalRot0Attr(Gf.Quatf(*[float(v) for v in quat]))
            j.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in pos1]))
            j.CreateLocalRot1Attr(Gf.Quatf(*[float(v) for v in quat1]))
            return j

        pos, quat = (x.numpy().astype(float) for x in bodies.get_world_poses())
        hi, fi = names.index(HANDS[held]), names.index(HANDS[free])
        anchor = fixed(paths[hi], pos[hi], quat[hi])
        for _ in range(2):
            app.update()
        lo, hi_lim = (x.numpy()[0] for x in art.get_dof_limits())
        clock = [0.0]
        frames = {"t": [], "poses": [], "phase": [], "q": [], "pelvis": [], "extras": {k: [] for k in extras}}
        xprims = {k: RigidPrim(v["path"]) for k, v in extras.items()}
        contacts = {}
        first_contact = [None]
        touch = {"hand_tool": False}

        def rot(q):
            w, x, y, z = q
            return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                             [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                             [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

        def momentum():
            p, q_ = (x.numpy().astype(float) for x in bodies.get_world_poses())
            lin, ang = (x.numpy().astype(float) for x in bodies.get_velocities())
            R = [rot(qq) for qq in q_]
            c = np.array([p[k] + R[k] @ com_local[k] for k in range(len(p))])
            C_ = (mass[:, None] * c).sum(axis=0) / mass.sum()
            P = (mass[:, None] * lin).sum(axis=0)
            L = sum(np.cross(c[k] - C_, mass[k] * lin[k]) + R[k] @ inertia[k] @ R[k].T @ ang[k] for k in range(len(p)))
            return P, L, C_

        def step(phase, hook=None):
            if hook:
                hook()
            SimulationManager.step(steps=1)
            clock[0] += dt
            headers, _ = sim.get_contact_report()
            for h in headers or []:
                a0, a1 = (str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1)))
                n0, n1 = a0.split("/")[-1], a1.split("/")[-1]
                if "Tool" in (n0, n1) and any(n.startswith(f"{free}_wrist") or n == HANDS[free] for n in (n0, n1)):
                    touch["hand_tool"] = True             # rubber-hand collision sits on the wrist yaw link
                if a0.startswith("/World/Environment") or a1.startswith("/World/Environment"):
                    actor = a1 if a0.startswith("/World/Environment") else a0
                    key = f"{phase}:{actor.split('/')[-1]}"
                    contacts[key] = contacts.get(key, 0) + 1
                    if first_contact[0] is None and phase == "drift" and actor.startswith(G1_PATH):
                        first_contact[0] = round(clock[0], 4)
            if round(clock[0] / dt) % round(1 / (30 * dt)) == 0:
                p, q_ = (x.numpy().astype(float) for x in bodies.get_world_poses())
                frames["t"].append(round(clock[0], 4))
                frames["poses"].append([p.round(5).tolist(), q_.round(6).tolist()])
                frames["phase"].append(phase)
                pi = names.index("pelvis")
                frames["pelvis"].append([p[pi].round(5).tolist(), q_[pi].round(6).tolist()])
                frames["q"].append({})
                for k, xp in xprims.items():
                    a, b = (x.numpy().astype(float)[0] for x in xp.get_world_poses())
                    frames["extras"][k].append([a.round(5).tolist(), b.round(6).tolist()])

        def track_hand(target_fn, phase, tol=0.75 * CAPTURE_R, max_s=30.0, until=None):
            """Resolved-rate free-hand tracking of a moving target with the free arm only (the held arm,
            legs and base are not driven, so the body stays put; including the base swung the whole body)."""
            cols = [6 + k for k, d in enumerate(dofs) if d.startswith(f"{free}_shoulder") or d.startswith(f"{free}_elbow")
                    or d.startswith(f"{free}_wrist")]
            var = cols
            qt = art.get_dof_positions().numpy()[0].copy()
            t_end = clock[0] + max_s
            hist = []
            while clock[0] < t_end:
                tgt, d = target_fn()
                hist.append(round(d, 4))
                if d <= tol or (until and until()):
                    return hist, True
                p = bodies.get_world_poses()[0].numpy().astype(float)
                e = tgt - p[fi]
                J = art.get_jacobian_matrices().numpy()[0].astype(float)
                A = J[fi, :3][:, var]
                dx = A.T @ np.linalg.solve(A @ A.T + 1e-3 * np.eye(3), 0.25 * e)
                dq = np.clip(dx, -0.02, 0.02)     # slow arm: the held wrist (5 N m) cannot hold back fast swings
                for k, c in enumerate(cols):
                    qt[c - 6] += dq[k]
                qt = np.clip(qt, lo, hi_lim)
                for _ in range(12):
                    art.set_dof_position_targets(qt[None].astype(np.float32))
                    step(phase, hooks[0])
            return hist, False

        hooks = [None]
        if args.demo == "handoff":
            # tool rides on Astrobee (scripted fixed joint, 0.25 m in front of its body); Astrobee starts 2.6 m
            # further along the aisle and flies to a release point 0.55 m in front of G1's free hand
            hp = pos[fi]
            pel_p = np.asarray(pel[0])
            fwd = np.array([1.0, 0, 0])
            # release in mid aisle ahead of the chest (the free hand still sits at the overhead handhold; a release
            # near the ceiling left Astrobee pinned against it)
            sh = pos[names.index(f"{free}_shoulder_pitch_link")]
            release = sh + np.array([0.75, 0.0, -0.10])
            ab_start = release + np.array([1.3, 0.0, 0.0])
            ab, tl_ = xprims["astrobee"], xprims["tool"]
            ab.set_world_poses(positions=[ab_start], orientations=[[1, 0, 0, 0]])
            ab.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
            tool_off = np.array([-0.25, 0.0, 0.0])
            tl_.set_world_poses(positions=[ab_start + tool_off], orientations=[[1, 0, 0, 0]])
            tl_.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
            carry = fixed("/World/Actors/Tool", tool_off, (1, 0, 0, 0), body0="/World/Actors/Astrobee")
            goal = {"p": ab_start.copy()}

            def fly():
                a, b = (x.numpy().astype(float)[0] for x in ab.get_world_poses())
                lv, av = (x.numpy().astype(float)[0] for x in ab.get_velocities())
                e = goal["p"] - a                          # velocity-capped approach (0.15 m/s), then hold
                v_des = e * min(1.0, 0.15 / max(np.linalg.norm(e), 1e-6)) if np.linalg.norm(e) > 0.15 else e * 1.0
                F = 9.0877 * 1.5 * (v_des - lv)
                if np.linalg.norm(F) > ASTROBEE_FORCE_N:
                    F *= ASTROBEE_FORCE_N / np.linalg.norm(F)
                w, x, y, z = b
                ax = np.array([x, y, z]) * (2.0 if w >= 0 else -2.0)        # small-angle orientation error
                T = -0.15 * ax - 0.4 * av
                ab.apply_forces_and_torques_at_pos(forces=[F], torques=[T])
            hooks[0] = fly
            path = [release + np.array([0.25, 0.0, 0.0])]
            for wp in path:                           # fly in, then settle at the release point
                goal["p"] = wp
                while True:
                    a = ab.get_world_poses()[0].numpy().astype(float)[0]
                    v = ab.get_velocities()[0].numpy().astype(float)[0]
                    step("astrobee_fly", fly)
                    if np.linalg.norm(wp - a) < 0.03 and np.linalg.norm(v) < 0.01 or clock[0] > 60:
                        break
            rec["astrobee_arrival_t_s"] = round(clock[0], 3)
            for _ in range(round(1.0 / dt)):
                step("astrobee_hold", fly)
            carry.GetPrim().CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(False)
            tp = tl_.get_world_poses()[0].numpy().astype(float)[0]
            v_rel = (sh + np.array([0.40, 0.0, -0.05]) - tp)          # drift toward the free arm's reach
            v_rel = 0.06 * v_rel / np.linalg.norm(v_rel)
            tl_.set_velocities(linear_velocities=[v_rel], angular_velocities=[[0.0, 0.0, 0.4]])
            rec["release"] = {"t_s": round(clock[0], 3), "tool_velocity_m_s": v_rel.round(4).tolist(),
                              "tool_spin_rad_s": [0, 0, 0.4], "note": "scripted release push"}
            goal["p"] = release + np.array([0.9, 0.35, -0.2])         # Astrobee backs away

            def tool_target():
                tq = tl_.get_world_poses()
                tpos, tquat = (x.numpy().astype(float)[0] for x in tq)
                tv = tl_.get_velocities()[0].numpy().astype(float)[0]
                axis = rot(tquat) @ np.array([1.0, 0, 0])
                h = bodies.get_world_poses()[0].numpy().astype(float)[fi]
                s = float(np.clip((h - tpos) @ axis, -0.13, 0.13))      # nearest point on the bar's grip span
                near = tpos + s * axis
                return near + 0.3 * tv, float(np.linalg.norm(h - near))     # lead 0.3 s
            # capture at the first PhysX contact between the free hand and the bar (else at hand-to-axis <= CAPTURE_R
            # + bar half-thickness): tracking to a distance threshold alone batted the bar away on contact
            hist, ok = track_hand(tool_target, "catch", tol=CAPTURE_R + TOOL["size_m"][1] / 2, max_s=args.seconds,
                                  until=lambda: touch["hand_tool"])
            rec["catch_distance_hist_m"] = hist[::5]
            if ok:
                hq = bodies.get_world_poses()
                hpos, hquat = (x.numpy().astype(float)[fi] for x in hq)
                tpos, tquat = (x.numpy().astype(float)[0] for x in tl_.get_world_poses())
                Rt = rot(tquat)
                local = Rt.T @ (hpos - tpos) / np.asarray(TOOL["size_m"])   # joint frames are in the scaled local space
                qrel = Gf.Quatf(*[float(v) for v in tquat]).GetInverse() * Gf.Quatf(*[float(v) for v in hquat])
                fixed(paths[fi], local, (qrel.GetReal(), *qrel.GetImaginary()), body0="/World/Actors/Tool")
                rec["catch"] = {"t_s": round(clock[0], 3), "hand_to_bar_axis_m": hist[-1], "status": "CAPTURED",
                                "trigger": "hand-tool contact" if touch["hand_tool"] else "distance"}
            else:
                rec["catch"] = {"t_s": round(clock[0], 3), "hand_to_bar_axis_m": hist[-1], "status": "FAIL_NO_CAPTURE"}
            for _ in range(round(3.0 / dt)):
                step("hold_tool", fly)
        else:
            # push: resolved rate on the measured state drives the pelvis along the aisle (+x) at PUSH_SPEED while the
            # held hand stays still in 6-D (floating-base Jacobian), for PUSH_S; then the handhold is released
            cols = [6 + k for k, d in enumerate(dofs) if not any(s in d for s in ("hip", "knee", "ankle"))]
            var = list(range(6)) + cols
            pi = names.index("pelvis")
            qb = art.get_dof_positions().numpy()[0].copy()
            for _ in range(round(PUSH_S / (12 * dt))):
                J = art.get_jacobian_matrices().numpy()[0].astype(float)
                A = np.vstack([J[pi, :3][:, var], J[hi][:, var]])
                dx = A.T @ np.linalg.solve(A @ A.T + 1e-4 * np.eye(9),
                                           np.r_[np.array([PUSH_SPEED, 0.0, 0.0]) * 12 * dt, np.zeros(6)])
                for k, c in enumerate(cols):
                    qb[c - 6] += dx[6 + k]
                qb = np.clip(qb, lo, hi_lim)
                for _ in range(12):
                    art.set_dof_position_targets(qb[None].astype(np.float32))
                    step("push")
            anchor.GetPrim().CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(False)
            rec["release_t_s"] = round(clock[0], 3)
            series = []
            for k in range(round(args.seconds / dt)):
                art.set_dof_position_targets(qb[None].astype(np.float32))
                step("drift")
                if k % 12 == 0:
                    P, L, Cm = momentum()
                    series.append({"t": round(clock[0], 3), "P": P.round(5).tolist(), "L": L.round(5).tolist(),
                                   "com": Cm.round(4).tolist()})
            rec["momentum_series"] = series
            fc = first_contact[0]
            pre = [s for s in series if fc is None or s["t"] < fc]
            if pre:
                P0, L0 = np.asarray(pre[0]["P"]), np.asarray(pre[0]["L"])
                rec["momentum_conservation"] = {
                    "window_s": [pre[0]["t"], pre[-1]["t"]], "first_contact_t_s": fc,
                    "P0_kg_m_s": P0.round(4).tolist(), "L0_kg_m2_s": L0.round(4).tolist(),
                    "max_dP_kg_m_s": round(max(float(np.linalg.norm(np.asarray(s["P"]) - P0)) for s in pre), 5),
                    "max_dL_kg_m2_s": round(max(float(np.linalg.norm(np.asarray(s["L"]) - L0)) for s in pre), 5),
                    "com_speed_m_s": round(float(np.linalg.norm(P0)) / float(mass.sum()), 4)}
        rec["contacts"] = contacts
        rec["sim_time_s"] = round(clock[0], 3)
        rec["frames"] = frames
        rec["link_paths"] = paths
        rec["total_steps"] = 1
        rec["segments"] = [{"step": 1, "status": (rec.get("catch") or {}).get("status", "DEMO"), "t_s": clock[0]}]
        timeline.stop()
    except Exception as exc:
        rec["error"] = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        out = OUT / f"{args.demo}_run.json"
        out.write_text(json.dumps(rec) + "\n", encoding="utf-8")
        summ = {k: rec.get(k) for k in ("demo", "error", "catch", "release", "astrobee_arrival_t_s", "release_t_s",
                                        "momentum_conservation", "sim_time_s")}
        summ["contact_events"] = sum((rec.get("contacts") or {}).values())
        (OUT / f"{args.demo}_summary.json").write_text(json.dumps(summ, indent=1), encoding="utf-8")
        print(json.dumps(summ), flush=True)
        app.close()


if __name__ == "__main__":
    main()

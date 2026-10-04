"""S1.4 Isaac Sim GUI walkthrough: the G1 worksite task simulated LIVE by PhysX in the GUI and screen-recorded.

Run inside the Isaac Sim GUI (after scripts/iss_physics_record.py has produced the IK/placement record):
  C:\\isaacsim\\isaac-sim.bat --exec "scripts/gui_demo_iss_physics.py" --/app/window/maximized=true
Every scene edit goes to the session layer (the scene file stays clean). The controller (posture PD, arm
interpolation, scripted fixed-joint grasp) runs in a physics-step callback; IK targets and the start pose are read
from outputs/iss_physics/physics_record.json. The cursor path and console lines are scripted. Only the Isaac Sim
window's client area is recorded. Output: outputs/iss_physics/gui_physics_raw.mp4 (gitignored).
"""
import asyncio
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import omni.kit.app
import omni.physx
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

ROOT = Path(r"D:\Workspace\06_Research\iss-lunar-physical-ai")
sys.path.insert(0, str(ROOT / "scripts"))
import gui_demo_iss_microlab as gm  # noqa: E402  (window pinning, cursor, capture helpers)
import iss_physics_common as C  # noqa: E402

REC = json.loads((C.OUT / "physics_record.json").read_text(encoding="utf-8"))
CFG = json.loads(C.GATE_CONFIG.read_text(encoding="utf-8"))["physics"]
RAW = C.OUT / "gui_physics_raw.mp4"


def log(msg):
    gm.carb.log_warn(f"[iss-s1.4] {msg}")


def smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


async def frames(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


def build(stage):
    """Session-layer edits: R2 off, G1 at the recorded start pose, gate-v3 physics, contact filtering."""
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        stage.GetPrimAtPath("/World/Actors/R2_Visual").SetActive(False)
        C.add_g1(stage, C.ROOT / REC["g1_usd"], tuple(REC["g1_translate"]))
        C.add_collision_groups(stage)
        scene = stage.GetPrimAtPath("/World/PhysicsScene")
        ps = PhysxSchema.PhysxSceneAPI.Apply(scene)
        ps.CreateTimeStepsPerSecondAttr(round(1.0 / CFG["dt_s"]))
        ps.CreateSolverTypeAttr(CFG["solver_type"])
        for p in Usd.PrimRange(stage.GetPrimAtPath(C.G1_PATH)):
            if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                a = PhysxSchema.PhysxArticulationAPI.Apply(p)
                a.CreateSolverPositionIterationCountAttr(CFG["position_iterations"])
                a.CreateSolverVelocityIterationCountAttr(CFG["velocity_iterations"])
                a.CreateEnabledSelfCollisionsAttr(False)
                a.CreateSleepThresholdAttr(0.0)
                root = str(p.GetPath())
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                b = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
                b.CreateLinearDampingAttr(0.0)
                b.CreateAngularDampingAttr(0.0)
                b.CreateSleepThresholdAttr(0.0)
    return root


class Controller:
    """Same task logic as iss_physics_record.py pass 2, driven from the PhysX step callback."""

    def __init__(self, root):
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        self.art = Articulation(root)
        names, self.paths = list(self.art.link_names), list(self.art.link_paths[0])
        self.bodies = RigidPrim(self.paths)
        dofs = list(self.art.dof_names)
        n = self.art.num_dofs
        self.art.set_dof_friction_properties(static_frictions=0.0, dynamic_frictions=0.0, viscous_frictions=0.0)
        self.art.set_dof_gains(np.full((1, n), C.POSTURE_GAINS[0], dtype=np.float32),
                               np.full((1, n), C.POSTURE_GAINS[1], dtype=np.float32))
        self.q0 = self.art.get_dof_positions().numpy()
        self.art.set_dof_position_targets(self.q0)
        self.art.set_dof_velocities(np.zeros((1, n), dtype=np.float32))
        normal = np.array([0.0, 1.0, 0.0])
        self.art.set_velocities(linear_velocities=[list(-normal * C.APPROACH_V)], angular_velocities=[[0, 0, 0]])
        self.arms = {s: {"idx": [dofs.index(d) for d in a["dofs"]], "t": a["t"],
                         "q": np.array(REC["ik"][s]["q_rad"]), "hand_i": names.index(a["hand"])}
                     for s, a in C.ARMS.items()}
        self.rail = np.array(REC["worksite"]["rail_pt"])
        self.t, self.prev_rd, self.grasp_request, self.grasped = 0.0, None, None, False

    def step(self, dt):
        q = self.q0.copy()
        for a in self.arms.values():
            u = smooth((self.t - a["t"][0]) / (a["t"][1] - a["t"][0]))
            q[0, a["idx"]] = self.q0[0, a["idx"]] + u * (a["q"] - self.q0[0, a["idx"]])
        self.art.set_dof_position_targets(q)
        if not self.grasped and self.grasp_request is None:
            pos, quat = (x.numpy().astype(float) for x in self.bodies.get_world_poses())
            i = self.arms["right"]["hand_i"]
            rd = float(np.linalg.norm(pos[i] - self.rail))
            if self.prev_rd is not None and self.prev_rd <= C.CAPTURE_R and rd > self.prev_rd:
                self.grasp_request = (pos[i], quat[i], rd, self.t)   # USD authoring happens on the app thread
            self.prev_rd = rd
        self.t += dt


async def main():
    stage_ctx = omni.usd.get_context()
    await stage_ctx.open_stage_async(str(C.SCENE))
    await frames(120)
    stage = stage_ctx.get_stage()
    root = build(stage)
    await frames(60)
    ui.Workspace.show_window("Console")
    con = ui.Workspace.get_window("Console")
    if con:
        con.focus()
    vp = get_active_viewport()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        cam = UsdGeom.Camera.Define(stage, "/GuiWorksiteCam")
        cam.CreateFocalLengthAttr(14.0)
        cam.CreateHorizontalApertureAttr(36.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 60.0))
        UsdGeom.Xformable(cam).AddTransformOp().Set(C.look_at((-0.75, 0.15, 0.45), (0.95, -0.75, 0.05)))
    vp.camera_path = "/GuiWorksiteCam"
    tl = omni.timeline.get_timeline_interface()
    tl.set_looping(False)
    tl.set_end_time(60.0)

    hwnd, _ = gm.kit_main_window()
    if hwnd is None:
        log("Kit main window not found; not recording")
        omni.kit.app.get_app().post_quit()
        return
    gm.HWND = hwnd
    gm.pin_window(hwnd, True)
    await asyncio.sleep(6)
    if gm.user32.GetForegroundWindow() != hwnd:
        log("Isaac Sim is not in the foreground; not recording")
        gm.pin_window(hwnd, False)
        omni.kit.app.get_app().post_quit()
        return
    org, rc = gm.wintypes.POINT(0, 0), gm.wintypes.RECT()
    gm.user32.ClientToScreen(hwnd, gm.ctypes.byref(org))
    gm.user32.GetClientRect(hwnd, gm.ctypes.byref(rc))
    w, h = (rc.right - rc.left) // 2 * 2, (rc.bottom - rc.top) // 2 * 2
    rec = subprocess.Popen([gm.FFMPEG, "-y", "-loglevel", "error", "-f", "gdigrab", "-framerate", "30", "-draw_mouse", "1",
                            "-offset_x", str(org.x), "-offset_y", str(org.y), "-video_size", f"{w}x{h}", "-i", "desktop",
                            "-c:v", "libx264", "-preset", "veryfast", "-crf", "18", "-pix_fmt", "yuv420p", str(RAW)],
                           stdin=subprocess.PIPE)
    try:
        await asyncio.sleep(1.0)
        log("open_stage('scenes/iss_microlab_v0.usda')   # NASA US Lab interior, truth-layer variants")
        await asyncio.sleep(0.8)
        log(f"R2_Visual.SetActive(False)   # kinematic only: R2 physics model fails the momentum gate")
        await asyncio.sleep(0.8)
        log(f"AddReference('{REC['g1_usd']}') -> {C.G1_PATH}   # Unitree G1, physics candidate (gate v3 PASS)")
        await asyncio.sleep(0.8)
        log(f"PhysicsScene: gravity 0, {CFG['solver_type']}, {round(1 / CFG['dt_s'])} Hz, "
            f"{CFG['position_iterations']}/{CFG['velocity_iterations']} iterations   # gate v3 settings")
        await gm.glide_cursor(gm.screen_xy("Stage", 0.30, 0.30))
        stage_ctx.get_selection().set_selected_prim_paths([C.G1_PATH], True)
        log(f"select {C.G1_PATH}")
        await asyncio.sleep(2.0)
        await gm.glide_cursor(gm.screen_xy("Property", 0.45, 0.35))
        await asyncio.sleep(1.5)
        stage_ctx.get_selection().clear_selected_prim_paths()
        await gm.glide_cursor(gm.screen_xy("Viewport", 0.018, 0.30))
        log("timeline.play()   # PhysX runs live; controller = posture PD + arm interpolation (physics-step callback)")
        tl.play()
        await frames(3)
        ctl = Controller(root)
        sub = omni.physx.get_physx_interface().subscribe_physics_step_events(ctl.step)
        await gm.glide_cursor(gm.screen_xy("Viewport", 0.55, 0.55))
        logged = set()
        while ctl.t < C.DURATION:
            if ctl.grasp_request is not None and not ctl.grasped:
                pos, quat, rd, t_ = ctl.grasp_request
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    j = UsdPhysics.FixedJoint.Define(stage, "/World/Actors/G1_RightHand_Anchor")
                    j.CreateBody1Rel().SetTargets([ctl.paths[ctl.arms["right"]["hand_i"]]])
                    j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
                    j.CreateLocalRot0Attr(Gf.Quatf(*[float(v) for v in quat]))
                    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
                    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
                ctl.grasped = True
                log(f"UsdPhysics.FixedJoint(world, right_rubber_hand) at t={t_:.2f} s, hand {rd * 100:.1f} cm from plan"
                    "   # scripted grasp, not contact")
            for label, t0 in (("right arm -> Handrail_B", C.ARMS["right"]["t"][0]),
                              ("left arm -> maintenance panel", C.ARMS["left"]["t"][0])):
                if ctl.t >= t0 and label not in logged:
                    logged.add(label)
                    log(f"t={ctl.t:.2f} s  reach: {label}")
            await frames(1)
        tl.pause()
        sub = None
        log(f"t={ctl.t:.2f} s  worksite posture held   # pause")
        await asyncio.sleep(1.5)
        # No truth-layer switch here: re-composing /World while the paused sim holds the session-layer collision
        # groups made PhysX log "Removing a CollisionGroup while the simulation is running" (first recording).
        # The affordance view is in the rendered three-panel clip instead.
    finally:
        rec.stdin.write(b"q")
        rec.stdin.flush()
        rec.wait(timeout=30)
        gm.pin_window(hwnd, False)
    log(f"recording saved: {RAW}")
    tl.stop()
    # Timeline end time and the bound viewport camera are written to the root layer by Kit; a first run left them
    # saved into the scene file. Discard root-layer edits before quitting so the generated scene stays untouched.
    stage.GetRootLayer().Reload(force=True)
    await asyncio.sleep(1.0)
    omni.kit.app.get_app().post_quit()


def _done(fut):
    if gm.HWND:
        gm.pin_window(gm.HWND, False)
    if fut.exception():
        log(f"walkthrough failed: {fut.exception()!r}")


asyncio.ensure_future(main()).add_done_callback(_done)

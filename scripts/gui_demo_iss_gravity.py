"""Isaac Sim GUI clip: the PhysicsScene gravity condition switched from microgravity (0) to ground (9.81 m/s^2), live.

Run inside the Isaac Sim GUI:
  C:\\isaacsim\\isaac-sim.bat --exec "scripts/gui_demo_iss_gravity.py" --/app/window/maximized=true
Opens scenes/iss_full_v0.usda; in the session layer only (the scene file stays clean) it drops six free probe cubes
into the US Lab aisle with small initial velocities, plays the timeline at gravity 0 (they drift), then selects
/World/PhysicsScene and sets physics:gravityMagnitude to 9.81 through ChangePropertyCommand, so the Property panel
shows the edit, and plays on (they fall to the deck). The Console lines are scripted; the viewport, panels and
PhysX are the real Isaac Sim UI, captured by Kit from its own window (no desktop capture, no cursor). Output: outputs/iss_activity/gui_gravity_raw.mp4 (gitignored).
"""
import asyncio
import shutil
import subprocess
import sys
from pathlib import Path

import omni.appwindow
import omni.kit.app
import omni.kit.renderer_capture
import omni.kit.commands
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

try:
    ROOT = Path(__file__).resolve().parents[1]
except NameError:          # some Kit --exec paths do not define __file__; launch from the repo root
    ROOT = Path.cwd()
sys.path.insert(0, str(ROOT / "scripts"))
import gui_demo_iss_microlab as gm  # noqa: E402  (window pinning, cursor, capture helpers)

SCENE = ROOT / "scenes" / "iss_full_v0.usda"
RAW = ROOT / "outputs" / "iss_activity" / "gui_gravity_raw.mp4"
SCENE_PATH = "/World/PhysicsScene"
FPS = 15
PROBES = [((2.0, -0.25, -4.55), (0.04, 0.02, 0.03)), ((2.3, 0.20, -4.35), (-0.03, 0.01, 0.02)),
          ((2.6, -0.10, -4.75), (0.02, -0.03, 0.04)), ((2.9, 0.25, -4.50), (-0.02, -0.02, -0.02)),
          ((3.2, -0.20, -4.30), (0.03, 0.03, 0.0)), ((3.5, 0.05, -4.65), (-0.04, 0.0, 0.03))]


def log(msg):
    gm.carb.log_warn(f"[iss-gravity] {msg}")


async def frames(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


def build(stage):
    stage.SetEditTarget(stage.GetSessionLayer())
    stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection("visual")
    for k, (p, v) in enumerate(PROBES):
        c = UsdGeom.Cube.Define(stage, f"/World/GravityProbes/Probe_{k}")
        c.CreateSizeAttr(0.12)
        c.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.45, 0.1)])
        UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(*p))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        rb = UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
        rb.CreateVelocityAttr(Gf.Vec3f(*v))
        UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(1.0)
    cam = UsdGeom.Camera.Define(stage, "/GuiGravityCam")
    cam.CreateFocalLengthAttr(14.0)
    cam.CreateHorizontalApertureAttr(36.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 60.0))
    UsdGeom.Xformable(cam).AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(Gf.Vec3d(0.6, -0.55, -4.45), Gf.Vec3d(2.9, 0.0, -4.95), Gf.Vec3d(0, 0, 1)).GetInverse())


async def main():
    ctx = omni.usd.get_context()
    await ctx.open_stage_async(str(SCENE))
    await frames(120)
    stage = ctx.get_stage()
    build(stage)
    await frames(60)
    ui.Workspace.show_window("Console")
    vp = get_active_viewport()
    vp.camera_path = "/GuiGravityCam"
    tl = omni.timeline.get_timeline_interface()
    tl.set_looping(False)
    tl.set_end_time(120.0)
    # Kit captures its own window (swapchain) frame by frame: no desktop capture, so no other window can be
    # recorded and focus/occlusion do not matter; the cursor is not moved or shown
    cap = omni.kit.renderer_capture.acquire_renderer_capture_interface()
    win = omni.appwindow.get_default_app_window()
    fdir = RAW.parent / "gui_gravity_frames"
    shutil.rmtree(fdir, ignore_errors=True)
    fdir.mkdir(parents=True)
    state = {"on": True, "n": 0}

    async def grab():
        loop = asyncio.get_event_loop()
        t0 = loop.time()
        while state["on"]:
            if loop.time() - t0 >= state["n"] / FPS:
                cap.capture_next_frame_swapchain(str(fdir / f"f{state['n']:05d}.png"), win)
                state["n"] += 1
            await omni.kit.app.get_app().next_update_async()
    grabber = asyncio.ensure_future(grab())
    attr = stage.GetPrimAtPath(SCENE_PATH).GetAttribute("physics:gravityMagnitude")
    try:
        await asyncio.sleep(1.0)
        log("open_stage('scenes/iss_full_v0.usda')   # 7-module ISS interior; six free probe cubes (session layer)")
        await asyncio.sleep(0.8)
        ctx.get_selection().set_selected_prim_paths([SCENE_PATH], True)
        log(f"select {SCENE_PATH}   gravityMagnitude = {attr.Get():g} m/s^2   # ISS microgravity (IVA, v0)")
        await asyncio.sleep(1.5)
        log("timeline.play()   # gravity 0: the cubes keep their small initial velocities and drift")
        tl.play()
        await asyncio.sleep(5.0)
        tl.pause()
        await asyncio.sleep(0.8)
        omni.kit.commands.execute("ChangePropertyCommand", prop_path=Sdf.Path(f"{SCENE_PATH}.physics:gravityMagnitude"),
                                  value=9.81, prev=attr.Get())
        log(f"ChangeProperty {SCENE_PATH}.physics:gravityMagnitude 0 -> {attr.Get():g} m/s^2   # ground condition")
        await asyncio.sleep(2.0)
        log("timeline.play()   # gravity 9.81 (-z): the same cubes fall to the deck")
        tl.play()
        await asyncio.sleep(5.0)
        tl.pause()
        await asyncio.sleep(1.5)
    finally:
        state["on"] = False
        await grabber
        cap.wait_async_capture(win)
    subprocess.run([gm.FFMPEG, "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(fdir / "f%05d.png"),
                    "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p",
                    str(RAW)], check=True)
    log(f"recording saved: {RAW} ({state['n']} frames)")
    tl.stop()
    stage.GetRootLayer().Reload(force=True)     # discard any root-layer edits Kit made; the scene file stays clean
    await asyncio.sleep(1.0)
    omni.kit.app.get_app().post_quit()


def _done(fut):
    if fut.exception():
        log(f"gravity clip failed: {fut.exception()!r}")


asyncio.ensure_future(main()).add_done_callback(_done)

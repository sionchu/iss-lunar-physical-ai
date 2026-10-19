"""Isaac Sim GUI clip: the PhysicsScene gravity condition switched from microgravity (0) to ground (9.81 m/s^2), live.

Run inside the Isaac Sim GUI:
  C:\\isaacsim\\isaac-sim.bat --exec "scripts/gui_demo_iss_gravity.py" --/app/window/maximized=true
Opens scenes/iss_full_v0.usda; in the session layer only (the scene file stays clean) it drops six free probe cubes
into the US Lab aisle with small initial velocities, plays the timeline at gravity 0 (they drift), then selects
/World/PhysicsScene and sets physics:gravityMagnitude to 9.81 through ChangePropertyCommand, so the Property panel
shows the edit, and plays on (they fall to the deck). The cursor path and Console lines are scripted; the
viewport, panels and PhysX are the real Isaac Sim UI. Output: outputs/iss_activity/gui_gravity_raw.mp4 (gitignored).
"""
import asyncio
import subprocess
import sys
from pathlib import Path

import omni.kit.app
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
    RAW.parent.mkdir(parents=True, exist_ok=True)
    rec = subprocess.Popen([gm.FFMPEG, "-y", "-loglevel", "error", "-f", "gdigrab", "-framerate", "30", "-draw_mouse", "1",
                            "-offset_x", str(org.x), "-offset_y", str(org.y), "-video_size", f"{w}x{h}", "-i", "desktop",
                            "-c:v", "libx264", "-preset", "veryfast", "-crf", "18", "-pix_fmt", "yuv420p", str(RAW)],
                           stdin=subprocess.PIPE)
    attr = stage.GetPrimAtPath(SCENE_PATH).GetAttribute("physics:gravityMagnitude")
    try:
        await asyncio.sleep(1.0)
        log("open_stage('scenes/iss_full_v0.usda')   # 7-module ISS interior; six free probe cubes (session layer)")
        await asyncio.sleep(0.8)
        await gm.glide_cursor(gm.screen_xy("Stage", 0.30, 0.20))
        ctx.get_selection().set_selected_prim_paths([SCENE_PATH], True)
        log(f"select {SCENE_PATH}   gravityMagnitude = {attr.Get():g} m/s^2   # ISS microgravity (IVA, v0)")
        await gm.glide_cursor(gm.screen_xy("Property", 0.55, 0.25))
        await asyncio.sleep(1.5)
        await gm.glide_cursor(gm.screen_xy("Viewport", 0.55, 0.55))
        log("timeline.play()   # gravity 0: the cubes keep their small initial velocities and drift")
        tl.play()
        await asyncio.sleep(5.0)
        tl.pause()
        await gm.glide_cursor(gm.screen_xy("Property", 0.55, 0.25))
        await asyncio.sleep(0.8)
        omni.kit.commands.execute("ChangePropertyCommand", prop_path=Sdf.Path(f"{SCENE_PATH}.physics:gravityMagnitude"),
                                  value=9.81, prev=attr.Get())
        log(f"ChangeProperty {SCENE_PATH}.physics:gravityMagnitude 0 -> {attr.Get():g} m/s^2   # ground condition")
        await asyncio.sleep(2.0)
        await gm.glide_cursor(gm.screen_xy("Viewport", 0.55, 0.55))
        log("timeline.play()   # gravity 9.81 (-z): the same cubes fall to the deck")
        tl.play()
        await asyncio.sleep(5.0)
        tl.pause()
        await asyncio.sleep(1.5)
    finally:
        rec.stdin.write(b"q")
        rec.stdin.flush()
        rec.wait(timeout=30)
        gm.pin_window(hwnd, False)
    log(f"recording saved: {RAW}")
    tl.stop()
    stage.GetRootLayer().Reload(force=True)     # discard any root-layer edits Kit made; the scene file stays clean
    await asyncio.sleep(1.0)
    omni.kit.app.get_app().post_quit()


def _done(fut):
    if gm.HWND:
        gm.pin_window(gm.HWND, False)
    if fut.exception():
        log(f"gravity clip failed: {fut.exception()!r}")


asyncio.ensure_future(main()).add_done_callback(_done)

"""Scripted Isaac Sim GUI walkthrough of the S1 scene, screen-recorded with ffmpeg (desktop capture incl. cursor).

Run inside the Isaac Sim GUI:
  C:\\isaacsim\\isaac-sim.bat --exec "scripts/gui_demo_iss_microlab.py" --/app/window/maximized=true
The cursor path and clicks are scripted (labelled as such in the receipt); the viewport, Stage/Property panels,
console and timeline playback are the real Isaac Sim UI. Output: outputs/iss_microlab_v0/gui_walkthrough_raw.mp4
"""
import asyncio
import ctypes
import math
import shutil
import subprocess
from ctypes import wintypes
from pathlib import Path

import carb
import omni.kit.app
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom

try:
    ROOT = Path(__file__).resolve().parents[1]
except NameError:          # some Kit --exec paths do not define __file__
    ROOT = Path(r"D:\Workspace\06_Research\iss-lunar-physical-ai")
SCENE = ROOT / "scenes" / "iss_microlab_v0.usda"
OUT = ROOT / "outputs" / "iss_microlab_v0"
FFMPEG = shutil.which("ffmpeg") or str(Path.home() / "AppData/Local/Microsoft/WinGet/Packages/"
                                        "Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe/ffmpeg-8.0-full_build/bin/ffmpeg.exe")
user32 = ctypes.windll.user32
user32.SetProcessDPIAware()
HWND = None


def log(msg):
    carb.log_warn(f"[iss-s1] {msg}")   # visible in the Console panel


async def frames(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


async def seconds(s):
    await asyncio.sleep(s)


def kit_main_window():
    """(hwnd, title) of this Kit process's visible top-level window."""
    found = []
    pid = ctypes.windll.kernel32.GetCurrentProcessId()

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def cb(hwnd, _):
        owner = wintypes.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(owner))
        if owner.value == pid and user32.IsWindowVisible(hwnd):
            buf = ctypes.create_unicode_buffer(256)
            user32.GetWindowTextW(hwnd, buf, 256)
            if buf.value.startswith("Isaac Sim"):
                found.append((hwnd, buf.value))
        return True

    user32.EnumWindows(cb, 0)
    return found[0] if found else (None, None)


def pin_window(hwnd, on):
    """Maximize + keep the Kit window on top while recording, so only Isaac Sim is captured."""
    HWND_TOPMOST, HWND_NOTOPMOST, SWP_NOMOVE, SWP_NOSIZE = -1, -2, 0x2, 0x1
    if on:
        user32.ShowWindow(hwnd, 3)                             # SW_MAXIMIZE
        user32.keybd_event(0x12, 0, 0, 0)                      # Alt tap lets SetForegroundWindow succeed
        user32.keybd_event(0x12, 0, 2, 0)
        user32.SetForegroundWindow(hwnd)
    user32.SetWindowPos(hwnd, HWND_TOPMOST if on else HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE)


def screen_xy(win_name, fx, fy):
    """Physical-pixel screen point at a fraction (fx, fy) of a docked Kit window (ui points * dpi scale)."""
    w = ui.Workspace.get_window(win_name)
    s = ui.Workspace.get_dpi_scale()
    if w is None or not w.visible:
        return None
    org = wintypes.POINT(0, 0)
    user32.ClientToScreen(HWND, ctypes.byref(org))           # ui window positions are client-area relative
    return org.x + int((w.position_x + fx * w.width) * s), org.y + int((w.position_y + fy * w.height) * s)


async def glide_cursor(target, dur=0.8):
    if target is None:
        return
    pt = wintypes.POINT()
    user32.GetCursorPos(ctypes.byref(pt))
    x0, y0 = pt.x, pt.y
    steps = max(2, int(dur * 60))
    for i in range(1, steps + 1):
        a = 0.5 - 0.5 * math.cos(math.pi * i / steps)
        user32.SetCursorPos(int(x0 + (target[0] - x0) * a), int(y0 + (target[1] - y0) * a))
        await asyncio.sleep(dur / steps)


def set_layer(stage, name):
    from pxr import Usd
    with Usd.EditContext(stage, stage.GetSessionLayer()):    # session layer: the scene file stays clean
        stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection(name)
    log(f'stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection("{name}")')


async def fly_camera(stage, vp, eye0, eye1, target, dur):
    """Session-layer perspective camera (never saved to the scene) moved through the aisle around R2."""
    from pxr import Usd
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        cam = UsdGeom.Camera.Define(stage, "/GuiDemoCam")
        cam.CreateFocalLengthAttr(16.0)
        cam.CreateHorizontalApertureAttr(36.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 60.0))
        op = UsdGeom.Xformable(cam).AddTransformOp()
    vp.camera_path = "/GuiDemoCam"
    steps = int(dur * 30)
    for i in range(steps + 1):
        a = 0.5 - 0.5 * math.cos(math.pi * i / steps)
        eye = [e0 + (e1 - e0) * a for e0, e1 in zip(eye0, eye1)]
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse())
        await asyncio.sleep(dur / steps)


async def main():
    OUT.mkdir(parents=True, exist_ok=True)
    ctx = omni.usd.get_context()
    await ctx.open_stage_async(str(SCENE))
    await frames(180)
    ui.Workspace.show_window("Console")                       # log lines below are the visible "command" trace
    console = ui.Workspace.get_window("Console")
    if console:
        console.focus()
    stage = ctx.get_stage()
    vp = get_active_viewport()
    vp.camera_path = "/World/Cameras/Hero"
    tl = omni.timeline.get_timeline_interface()
    tl.set_looping(False)
    tl.set_end_time(8.0)
    tl.set_current_time(0.0)
    global HWND
    HWND, title = kit_main_window()
    if HWND is None:
        log("Kit main window not found; not recording (never fall back to full-desktop capture)")
        omni.kit.app.get_app().post_quit()
        return
    pin_window(HWND, True)
    await seconds(6)                                          # textures + RTX accumulation settle
    if user32.GetForegroundWindow() != HWND:
        log("Kit window is not in the foreground; not recording")
        pin_window(HWND, False)
        omni.kit.app.get_app().post_quit()
        return
    raw = OUT / "gui_walkthrough_raw.mp4"
    # Capture only the screen region of the Isaac Sim client area (window pinned on top), never the whole desktop.
    # (gdigrab title= returns a stale image for the Vulkan viewport, so grab the composed desktop region instead.)
    org, rc = wintypes.POINT(0, 0), wintypes.RECT()
    user32.ClientToScreen(HWND, ctypes.byref(org))
    user32.GetClientRect(HWND, ctypes.byref(rc))
    w, h = (rc.right - rc.left) // 2 * 2, (rc.bottom - rc.top) // 2 * 2
    log(f"recording client area {w}x{h} at ({org.x},{org.y})")
    rec = subprocess.Popen([FFMPEG, "-y", "-loglevel", "error", "-f", "gdigrab", "-framerate", "30", "-draw_mouse", "1",
                            "-offset_x", str(org.x), "-offset_y", str(org.y), "-video_size", f"{w}x{h}", "-i", "desktop",
                            "-c:v", "libx264", "-preset", "veryfast", "-crf", "18", "-pix_fmt", "yuv420p",
                            str(raw)], stdin=subprocess.PIPE)
    await seconds(1.5)
    log("Opened scenes/iss_microlab_v0.usda  |  NASA US Lab interior (nasa/astrobee_media)  |  R2 = KINEMATIC / VISUAL")
    await glide_cursor(screen_xy("Stage", 0.30, 0.40))
    ctx.get_selection().set_selected_prim_paths(["/World/Actors/R2_Visual"], True)
    log("Selected /World/Actors/R2_Visual -> iss:classification = KINEMATIC_VISUAL_ONLY (zero-g Test B: FAIL)")
    await seconds(2.0)
    await glide_cursor(screen_xy("Property", 0.45, 0.40))
    await seconds(2.5)
    ctx.get_selection().clear_selected_prim_paths()
    await glide_cursor(screen_xy("Viewport", 0.018, 0.30))   # left viewport toolbar (play controls)
    log("timeline.play()  ->  8 s kinematic keyframe clip @ 30 fps (authored motion, not a physics result)")
    tl.play()
    await seconds(8.5)
    tl.pause()
    await glide_cursor(screen_xy("Viewport", 0.55, 0.55))
    for layer, hold in (("collision", 3.0), ("affordance", 3.0), ("visual", 1.5)):
        set_layer(stage, layer)
        await seconds(hold)
    log("Viewport camera fly-through (session layer only)")
    await fly_camera(stage, vp, (-2.6, 0.55, 0.35), (-0.55, -0.55, 0.30), (0.75, 0.0, 0.05), 5.0)
    await seconds(1.0)
    vp.camera_path = "/World/Cameras/Hero"
    await seconds(1.5)
    rec.stdin.write(b"q")
    rec.stdin.flush()
    rec.wait(timeout=30)
    pin_window(HWND, False)
    log(f"recording saved: {raw}")
    await seconds(1.0)
    omni.kit.app.get_app().post_quit()


def _done(fut):
    if HWND:
        pin_window(HWND, False)                              # never leave the window pinned on top
    if fut.exception():
        log(f"walkthrough failed: {fut.exception()!r}")


if __name__ == "__main__":   # helpers are reused by gui_demo_iss_physics.py
    asyncio.ensure_future(main()).add_done_callback(_done)

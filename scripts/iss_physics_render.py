"""S1.4: replay the recorded PhysX worksite run and render a 3-panel 1920x1080 clip (RTX path tracing).

C:\\isaacsim\\python.bat scripts/iss_physics_render.py [--quick] [--subframes=N]
Panels: (1) R2 kinematic clip, (2) G1 worksite task (physics replay), (3) same replay in the affordance layer.
Nothing is re-simulated: poses come from outputs/iss_physics/physics_record.json; physics is disabled on the replica.
"""
import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--quick", action="store_true")
ap.add_argument("--subframes", type=int, default=32)
ap.add_argument("--resume", action="store_true", help="keep existing frames and render only the missing ones")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 1080})   # before any pxr import

sys.path.insert(0, str(Path(__file__).resolve().parent))
import iss_physics_common as C  # noqa: E402

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402

REC = json.loads((C.OUT / "physics_record.json").read_text(encoding="utf-8"))
if REC.get("error"):
    raise SystemExit(f"physics record has an error: {REC['error']}")
FR = REC["frames"]
R2_CLIP_END = 240

s = carb.settings.get_settings()
for k, v in (("/rtx/rendermode", "PathTracing"), ("/rtx/pathtracing/spp", 1),
             ("/rtx/pathtracing/totalSpp", args.subframes), ("/rtx/pathtracing/optixDenoiser/enabled", True),
             ("/rtx/post/motionblur/enabled", False), ("/rtx/post/dof/enabled", False),
             ("/rtx/post/lensFlares/enabled", False), ("/rtx/post/histogram/enabled", False)):
    s.set(k, v)

ctx = omni.usd.get_context()
ctx.open_stage(str(C.SCENE))
stage = ctx.get_stage()
g1_usd = C.ROOT / REC["g1_usd"]
C.add_g1(stage, g1_usd, tuple(REC["g1_translate"]))
for _ in range(30):
    app.update()
for root in (C.G1_PATH,):   # replica only: no simulation
    # de-instance: RTX kept instanced hand meshes visible when the actor root was hidden per panel (seen in QA)
    for p in [p for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.IsInstance()]:
        p.SetInstanceable(False)
    for p in list(Usd.PrimRange(stage.GetPrimAtPath(root))):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
        if p.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            PhysxSchema.PhysxArticulationAPI.Apply(p).CreateArticulationEnabledAttr(False)

cam = UsdGeom.Camera.Define(stage, "/World/Cameras/Panel")
cam.CreateHorizontalApertureAttr(36.0 * C.PANEL_W / C.PANEL_H)
cam.CreateVerticalApertureAttr(36.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 100.0))
cam_op = UsdGeom.Xformable(cam).AddTransformOp()
rp = rep.create.render_product(str(cam.GetPath()), (C.PANEL_W, C.PANEL_H))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach(rp)
rep.orchestrator.set_capture_on_play(False)
timeline = omni.timeline.get_timeline_interface()
world = stage.GetPrimAtPath("/World")
vis = {k: UsdGeom.Imageable(stage.GetPrimAtPath(p)) for k, p in
       (("r2", "/World/Actors/R2_Visual"), ("g1", C.G1_PATH))}
SHOW = {"r2": "r2", "g1": "g1", "aff": "g1"}      # panel key -> visible actor

# ---- pose replay -------------------------------------------------------------------------------------------------
cache = UsdGeom.XformCache()
PATHS = FR["paths"]
PARENT = {p: str(stage.GetPrimAtPath(p).GetParent().GetPath()) for p in PATHS}
STATIC = {p: cache.GetLocalToWorldTransform(stage.GetPrimAtPath(q)) for p, q in PARENT.items() if q not in PATHS}


def world_m(pos, q):
    m = Gf.Matrix4d()
    m.SetRotate(Gf.Quatd(q[0], q[1], q[2], q[3]))
    m.SetTranslateOnly(Gf.Vec3d(*pos))
    return m


def set_local(prim, m):
    xf = Gf.Transform(m)
    t, r = xf.GetTranslation(), xf.GetRotation().GetQuat()
    x = UsdGeom.Xformable(prim)
    ops = {o.GetOpName(): o for o in x.GetOrderedXformOps()}
    if "xformOp:translate" not in ops:
        ops["xformOp:translate"] = x.AddTranslateOp()
    if "xformOp:orient" not in ops:
        ops["xformOp:orient"] = x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    ops["xformOp:translate"].Set(Gf.Vec3d(t))
    o = ops["xformOp:orient"]
    o.Set(Gf.Quatf(r) if o.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Quatd(r))


def apply_frame(i):
    pos, quat = FR["poses"][i]
    worlds = {p: world_m(pos[k], quat[k]) for k, p in enumerate(PATHS)}
    for p in PATHS:
        par = worlds.get(PARENT[p], STATIC.get(p))
        set_local(stage.GetPrimAtPath(p), worlds[p] * par.GetInverse())


def render_panel(key, layer, eye, target, focal, frame):
    world.GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection(layer)
    for k, im in vis.items():
        im.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if k == SHOW[key] else UsdGeom.Tokens.invisible)
    cam.GetFocalLengthAttr().Set(focal)
    cam_op.Set(C.look_at(eye, target))
    timeline.set_current_time(min(frame, R2_CLIP_END) / C.FPS)
    app.update()
    rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0, pause_timeline=True)
    return Image.fromarray(np.asarray(rgb.get_data())[:, :, :3].astype(np.uint8), "RGB")


# ---- overlay -----------------------------------------------------------------------------------------------------
FD = Path("C:/Windows/Fonts")
F_NAME = ImageFont.truetype(str(FD / "seguisb.ttf"), 34)
F_TAG = ImageFont.truetype(str(FD / "bahnschrift.ttf"), 17)
F_TXT = ImageFont.truetype(str(FD / "segoeui.ttf"), 18)
TAGS = {"r2": ("R2", "KINEMATIC / VISUAL"), "g1": ("G1 · WORKSITE", "PHYSICS · PhysX PGS"),
        "aff": ("TASK TARGETS", "AFFORDANCE LAYER")}
G = REC["grasp"]
PHASE = {"drift": "drifting toward Handrail_B", "reach_rail": "right hand reaching the rail",
         "anchored": "anchored on rail (fixed-joint grasp)", "reach_panel": "left hand reaching the panel",
         "at_panel": "left hand at panel (no contact)"}


def lines(key, i):
    t, ph = FR["t"][i], FR["phase"][i]
    if key == "r2":
        return ["Authored keyframe clip, no dynamics", "R2 physics model fails the momentum gate",
                "→ physics shown with G1 (next panels)"]
    if key == "g1":
        return [f"t = {t:4.1f} s   zero g   {PHASE[ph]}",
                f"right hand → rail {FR['right_d'][i] * 100:4.1f} cm   left → panel {FR['left_d'][i] * 100:4.1f} cm",
                "Grasp = scripted constraint, no contact physics"]
    return ["Teal: graspable handrails   Blue: panel", "What the task layer exposes to the robot",
            "Same PhysX replay, affordance view"]


def overlay(img, key, i):
    base = img.convert("RGBA")
    ov = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    m = 28
    name, tag = TAGS[key]
    d.rectangle([0, 0, C.PANEL_W, 92], fill=(10, 12, 16, 150))
    d.text((m, 22), name, font=F_NAME, fill=(255, 255, 255, 240))
    tw = d.textlength(tag, font=F_TAG)
    x0 = C.PANEL_W - m - tw - 20
    d.rectangle([x0, 32, x0 + tw + 20, 62], outline=(255, 255, 255, 200), width=1)
    d.text((x0 + 10, 37), tag, font=F_TAG, fill=(255, 255, 255, 235))
    ls = lines(key, i)
    y0 = C.PANEL_H - 30 - 28 * len(ls)
    d.rectangle([0, y0 - 18, C.PANEL_W, C.PANEL_H], fill=(10, 12, 16, 150))
    for k, s_ in enumerate(ls):
        d.text((m, y0 + 28 * k), s_, font=F_TXT, fill=(255, 255, 255, 230))
    return Image.alpha_composite(base, ov).convert("RGB")


def compose(panels):
    out = Image.new("RGB", (C.PANEL_W * 3, C.PANEL_H), (8, 8, 10))
    for k, p in enumerate(panels):
        out.paste(p, (k * C.PANEL_W, 0))
    d = ImageDraw.Draw(out)
    for k in (1, 2):
        d.rectangle([k * C.PANEL_W - 2, 0, k * C.PANEL_W + 1, C.PANEL_H], fill=(8, 8, 10))
    return out


def main():
    t0 = time.time()
    frames_dir = C.OUT / ("frames_quick" if args.quick else "frames")
    if not args.resume:
        shutil.rmtree(frames_dir, ignore_errors=True)
    frames_dir.mkdir(parents=True, exist_ok=True)
    idx = [0, 60, 150, 300] if args.quick else list(range(len(FR["t"])))
    skipped = 0
    for n, i in enumerate(idx):
        if args.resume and (frames_dir / f"f{i:04d}.png").exists():
            skipped += 1
            continue
        apply_frame(i)
        panels = [overlay(render_panel(key, layer, eye, tgt, foc, i), key, i) for key, layer, eye, tgt, foc in C.PANELS]
        compose(panels).save(frames_dir / f"f{i:04d}.png")
        if n % 30 == 0:
            print(f"frame {i} {time.time() - t0:.0f}s", flush=True)
    receipt = {"record": "outputs/iss_physics/physics_record.json", "subframes": args.subframes, "frames": len(idx),
               "panel": [C.PANEL_W, C.PANEL_H], "panels": C.PANELS, "resumed_skipped_frames": skipped, "seconds": round(time.time() - t0, 1)}
    if not args.quick:
        shutil.copy(frames_dir / "f0240.png", C.OUT / "iss_physics_three_panel_still.png")
        mp4 = C.OUT / "iss_physics_three_panel_1080p30.mp4"
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(C.FPS), "-i", str(frames_dir / "f%04d.png"),
                        "-c:v", "libx264", "-preset", "slow", "-crf", "17", "-pix_fmt", "yuv420p",
                        "-movflags", "+faststart", str(mp4)], check=True)
        receipt["video"] = str(mp4.relative_to(C.ROOT))
    (C.OUT / f"render_receipt{'_quick' if args.quick else ''}.json").write_text(json.dumps(receipt, indent=1), encoding="utf-8")
    print(json.dumps(receipt), flush=True)


try:
    main()
finally:
    app.close()

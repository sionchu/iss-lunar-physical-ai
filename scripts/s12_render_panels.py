"""S1.2: replay the recorded PhysX run in the ISS scene and render a 3-panel 1920x1080 clip (RTX path tracing).

C:\\isaacsim\\python.bat scripts/s12_render_panels.py [--quick] [--subframes N]
Panels: (1) R2 kinematic clip, (2) G1 physics replay, (3) cargo physics replay in the collision layer.
Physics is NOT re-simulated here: poses come from outputs/s12/physics_record.json, physics is disabled on the replica.
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
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 1080})   # before any pxr import

sys.path.insert(0, str(Path(__file__).resolve().parent))
from s12_common import (CARGO_PATH, FPS, G1_PATH, OUT, PANEL_H, PANEL_W, PANELS, ROOT, SCENE,  # noqa: E402
                        add_cargo, add_g1, look_at)

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402

REC = json.loads((OUT / "physics_record.json").read_text(encoding="utf-8"))
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
ctx.open_stage(str(SCENE))
stage = ctx.get_stage()
add_g1(stage, ROOT / REC["g1_usd"])
add_cargo(stage, physics=False)
for _ in range(30):
    app.update()
# replica only: no simulation of G1 (poses are replayed from the record)
for p in list(Usd.PrimRange(stage.GetPrimAtPath(G1_PATH))):
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
    if p.IsA(UsdPhysics.Joint):
        UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        PhysxSchema.PhysxArticulationAPI.Apply(p).CreateArticulationEnabledAttr(False)

cam = UsdGeom.Camera.Define(stage, "/World/Cameras/Panel")
cam.CreateHorizontalApertureAttr(36.0 * PANEL_W / PANEL_H)
cam.CreateVerticalApertureAttr(36.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 100.0))
cam_op = UsdGeom.Xformable(cam).AddTransformOp()
rp = rep.create.render_product(str(cam.GetPath()), (PANEL_W, PANEL_H))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach(rp)
rep.orchestrator.set_capture_on_play(False)
timeline = omni.timeline.get_timeline_interface()
world = stage.GetPrimAtPath("/World")
vis = {k: UsdGeom.Imageable(stage.GetPrimAtPath(p)) for k, p in
       (("r2", "/World/Actors/R2_Visual"), ("g1", G1_PATH), ("cargo", CARGO_PATH))}

# ---- pose replay -------------------------------------------------------------------------------------------------
cache = UsdGeom.XformCache()
g1_paths, cargo_paths = FR["g1_paths"], FR["cargo_paths"]
parent_of = {p: str(stage.GetPrimAtPath(p).GetParent().GetPath()) for p in g1_paths}
static_parent = {p: cache.GetLocalToWorldTransform(stage.GetPrimAtPath(q)) for p, q in parent_of.items() if q not in g1_paths}


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
    gp, gq = FR["g1"][i]
    worlds = {p: world_m(gp[k], gq[k]) for k, p in enumerate(g1_paths)}
    for p in g1_paths:
        parent = worlds.get(parent_of[p], static_parent.get(p))
        set_local(stage.GetPrimAtPath(p), worlds[p] * parent.GetInverse())
    cp, cq = FR["cargo"][i]
    for k, p in enumerate(cargo_paths):
        set_local(stage.GetPrimAtPath(p), world_m(cp[k], cq[k]))   # parent /World/Actors/Cargo is identity


def render_panel(key, layer, eye, target, focal, frame):
    world.GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection(layer)
    for k, im in vis.items():
        im.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if k == key else UsdGeom.Tokens.invisible)
    cam.GetFocalLengthAttr().Set(focal)
    cam_op.Set(look_at(eye, target))
    timeline.set_current_time(min(frame, R2_CLIP_END) / FPS)
    app.update()
    rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0, pause_timeline=True)
    return Image.fromarray(np.asarray(rgb.get_data())[:, :, :3].astype(np.uint8), "RGB")


# ---- overlay -----------------------------------------------------------------------------------------------------
FD = Path("C:/Windows/Fonts")
F_NAME = ImageFont.truetype(str(FD / "seguisb.ttf"), 34)
F_TAG = ImageFont.truetype(str(FD / "bahnschrift.ttf"), 17)
F_TXT = ImageFont.truetype(str(FD / "segoeui.ttf"), 18)
TAGS = {"r2": ("R2", "KINEMATIC / VISUAL"), "g1": ("G1", "PHYSICS · PhysX PGS"), "cargo": ("CARGO", "PHYSICS · RIGID BODIES")}
GM = REC["g1_momentum"]


def lines(key, i):
    t = FR["t"][i]
    if key == "r2":
        return ["Authored keyframe clip, no dynamics", "NASA US Lab interior (astrobee_media)", "Isaac Sim 6.0.1 · RTX"]
    if key == "g1":
        r = GM["per_frame_angular_ratio_running_max"][i]
        return [f"t = {t:4.1f} s   free-floating, zero g", "Arm PD sine (gate Test B), contacts filtered",
                f"|ΔL| / Lref = {r:.4f}   gate ≤ {GM['angular_tolerance']:.2f}"]
    return [f"t = {t:4.1f} s   3 × 10 kg boxes, zero g", "PhysX contacts with collision proxies",
            "Collision-layer view · restitution 0.2"]


def overlay(img, key, i):
    base = img.convert("RGBA")
    ov = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    m = 28
    name, tag = TAGS[key]
    d.rectangle([0, 0, PANEL_W, 92], fill=(10, 12, 16, 150))
    d.text((m, 22), name, font=F_NAME, fill=(255, 255, 255, 240))
    tw = d.textlength(tag, font=F_TAG)
    x0 = PANEL_W - m - tw - 20
    d.rectangle([x0, 32, x0 + tw + 20, 62], outline=(255, 255, 255, 200), width=1)
    d.text((x0 + 10, 37), tag, font=F_TAG, fill=(255, 255, 255, 235))
    ls = lines(key, i)
    y0 = PANEL_H - 30 - 28 * len(ls)
    d.rectangle([0, y0 - 18, PANEL_W, PANEL_H], fill=(10, 12, 16, 150))
    for k, s_ in enumerate(ls):
        d.text((m, y0 + 28 * k), s_, font=F_TXT, fill=(255, 255, 255, 230))
    return Image.alpha_composite(base, ov).convert("RGB")


def compose(panels):
    gap = 0
    out = Image.new("RGB", (PANEL_W * 3, PANEL_H), (8, 8, 10))
    for k, p in enumerate(panels):
        out.paste(p, (k * PANEL_W, 0))
    d = ImageDraw.Draw(out)
    for k in (1, 2):
        d.rectangle([k * PANEL_W - 2, 0, k * PANEL_W + 1, PANEL_H], fill=(8, 8, 10))
    return out


def main():
    t0 = time.time()
    frames_dir = OUT / ("frames_quick" if args.quick else "frames")
    shutil.rmtree(frames_dir, ignore_errors=True)
    frames_dir.mkdir(parents=True)
    idx = [0, 150, 300] if args.quick else list(range(len(FR["t"])))
    for n, i in enumerate(idx):
        apply_frame(i)
        panels = [overlay(render_panel(key, layer, eye, tgt, foc, i), key, i) for key, layer, eye, tgt, foc in PANELS]
        compose(panels).save(frames_dir / f"f{i:04d}.png")
        if n % 30 == 0:
            print(f"frame {i} {time.time() - t0:.0f}s", flush=True)
    receipt = {"record": "outputs/s12/physics_record.json", "subframes": args.subframes, "frames": len(idx),
               "panel": [PANEL_W, PANEL_H], "panels": PANELS, "seconds": round(time.time() - t0, 1)}
    if not args.quick:
        shutil.copy(frames_dir / "f0180.png", OUT / "s12_three_panel_still.png")
        mp4 = OUT / "s12_three_panel_1080p30.mp4"
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(frames_dir / "f%04d.png"),
                        "-c:v", "libx264", "-preset", "slow", "-crf", "17", "-pix_fmt", "yuv420p",
                        "-movflags", "+faststart", str(mp4)], check=True)
        receipt["video"] = str(mp4.relative_to(ROOT))
    (OUT / f"render_receipt{'_quick' if args.quick else ''}.json").write_text(json.dumps(receipt, indent=1), encoding="utf-8")
    print(json.dumps(receipt), flush=True)


try:
    main()
finally:
    app.close()

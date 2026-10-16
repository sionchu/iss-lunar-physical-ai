"""Replay the recorded full-ISS G1 traverse (iss_activity_run.py --record) and render a chase-camera clip.

C:\\isaacsim\\python.bat scripts/render_iss_activity.py --run outputs/iss_activity/chain_online_run.json
    [--every 2] [--subframes 16] [--width 1280 --height 720]
Nothing is re-simulated: link poses come from the run record; physics is disabled on the replica. One camera,
one render product (retargeting a render product between cameras overexposed later frames, see the S2 receipt).
Overlay: physics label, step / status from the record, and the standing caveats (scripted grasps, placeholder
handholds, contacts filtered). Output: outputs/iss_activity/ (gitignored).
"""
import argparse
import json
import shutil
import subprocess
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--run", type=Path, required=True)
ap.add_argument("--every", type=int, default=2, help="render every Nth recorded frame (recorded at 30 fps)")
ap.add_argument("--subframes", type=int, default=16)
ap.add_argument("--width", type=int, default=1280)
ap.add_argument("--height", type=int, default=720)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": args.width, "height": args.height})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "iss_activity"
REC = json.loads(args.run.read_text(encoding="utf-8"))
FR = REC["frames"]
G1_PATH = "/World/Actors/G1"

s = carb.settings.get_settings()
for k, v in (("/rtx/rendermode", "PathTracing"), ("/rtx/pathtracing/spp", 1),
             ("/rtx/pathtracing/totalSpp", args.subframes), ("/rtx/pathtracing/optixDenoiser/enabled", True),
             ("/rtx/post/motionblur/enabled", False), ("/rtx/post/dof/enabled", False),
             ("/rtx/post/lensFlares/enabled", False), ("/rtx/post/histogram/enabled", False)):
    s.set(k, v)

ctx = omni.usd.get_context()
ctx.open_stage(str(ROOT / REC["scene"]))
stage = ctx.get_stage()
g1_usd = next(Path(str(args.run.resolve().with_suffix("")) + "_g1").rglob("*.usd*"))
stage.DefinePrim(G1_PATH, "Xform").GetReferences().AddReference(str(g1_usd))
for _ in range(30):
    app.update()
for p in [p for p in Usd.PrimRange(stage.GetPrimAtPath(G1_PATH)) if p.IsInstance()]:
    p.SetInstanceable(False)
for p in list(Usd.PrimRange(stage.GetPrimAtPath(G1_PATH))):
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
    if p.IsA(UsdPhysics.Joint):
        UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        PhysxSchema.PhysxArticulationAPI.Apply(p).CreateArticulationEnabledAttr(False)

PATHS = REC["link_paths"]
PARENT = {p: str(stage.GetPrimAtPath(p).GetParent().GetPath()) for p in PATHS}
cache = UsdGeom.XformCache()
STATIC = {p: cache.GetLocalToWorldTransform(stage.GetPrimAtPath(q)) for p, q in PARENT.items() if q not in PATHS}


def world_m(pos, q):
    m = Gf.Matrix4d()
    m.SetRotate(Gf.Quatd(q[0], q[1], q[2], q[3]))
    m.SetTranslateOnly(Gf.Vec3d(*pos))
    return m


def set_local(prim, m):
    xf = Gf.Transform(m)
    x = UsdGeom.Xformable(prim)
    ops = {o.GetOpName(): o for o in x.GetOrderedXformOps()}
    t = ops.get("xformOp:translate") or x.AddTranslateOp()
    o = ops.get("xformOp:orient") or x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    t.Set(Gf.Vec3d(xf.GetTranslation()))
    r = xf.GetRotation().GetQuat()
    o.Set(Gf.Quatf(r) if o.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Quatd(r))


def apply_frame(i):
    pos, quat = FR["poses"][i]
    worlds = {p: world_m(pos[k], quat[k]) for k, p in enumerate(PATHS)}
    for p in PATHS:
        par = worlds.get(PARENT[p], STATIC.get(p))
        set_local(stage.GetPrimAtPath(p), worlds[p] * par.GetInverse())


cam = UsdGeom.Camera.Define(stage, "/World/Cameras/Chase")
cam.CreateFocalLengthAttr(14.0)
cam.CreateHorizontalApertureAttr(36.0)
cam.CreateVerticalApertureAttr(36.0 * args.height / args.width)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 80.0))
cam_op = UsdGeom.Xformable(cam).AddTransformOp()
rp = rep.create.render_product(str(cam.GetPath()), (args.width, args.height))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach(rp)
rep.orchestrator.set_capture_on_play(False)
stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection("visual")

pel = np.array([p for p, _ in FR["pelvis"]])
# chase camera: 1.6 m behind the smoothed pelvis along the smoothed travel direction, 0.25 m below (aisle side)
K = 45
smooth = np.array([pel[max(0, i - K):i + K + 1].mean(axis=0) for i in range(len(pel))])
step_at = []                                 # step number / status per frame from the segment end times
ends = [(s_.get("t_s", 1e9), s_["step"], s_["status"]) for s_ in REC["segments"]]
font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", int(22 * args.width / 1280))
small = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", int(17 * args.width / 1280))
frames_dir = OUT / "render_frames"
shutil.rmtree(frames_dir, ignore_errors=True)
frames_dir.mkdir(parents=True)
n_out = 0
for i in range(0, len(FR["t"]), args.every):
    apply_frame(i)
    j0, j1 = max(0, i - 30), min(len(smooth) - 1, i + 30)
    d = smooth[j1] - smooth[j0]
    d = d / np.linalg.norm(d) if np.linalg.norm(d) > 1e-3 else np.array([1.0, 0, 0])
    eye = smooth[i] - 1.6 * d + np.array([0, 0, -0.25])
    cam_op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*(smooth[i] + 0.4 * d)), Gf.Vec3d(0, 0, 1)).GetInverse())
    app.update()
    rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0, pause_timeline=True)
    img = Image.fromarray(np.asarray(rgb.get_data())[:, :, :3].astype(np.uint8), "RGB").convert("RGBA")
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    t = FR["t"][i]
    done = [e for e in ends if e[0] <= t]
    nxt = next((e for e in ends if e[0] > t), None)
    status = f"step {nxt[1]} of {REC.get('total_steps')}" if nxt else (f"step {done[-1][1]}: {done[-1][2]}" if done else "")
    lines = [("G1 · PhysX (gate v3 physics) · hand-over-hand, closed-loop cuRobo plans", font),
             (f"t = {t:5.1f} s   {status}   captures so far: {sum(e[2] == 'CAPTURED' for e in done)}", font),
             ("scripted grasps (fixed joints) · PLACEHOLDER handholds outside the S1 rail and hatch rims · "
              "G1-ISS contacts filtered", small)]
    y = int(18 * args.width / 1280)
    for text, f in lines:
        dr.text((21, y + 1), text, font=f, fill=(0, 0, 0, 170))
        dr.text((20, y), text, font=f, fill=(255, 255, 255, 235))
        y += int(f.size * 1.4)
    Image.alpha_composite(img, ov).convert("RGB").save(frames_dir / f"f{n_out:05d}.png")
    n_out += 1
mp4 = OUT / "iss_activity_traverse.mp4"
fps = 30 // args.every
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", str(frames_dir / "f%05d.png"),
                "-c:v", "libx264", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)], check=True)
print(json.dumps({"video": str(mp4), "frames": n_out, "fps": fps, "run": str(args.run)}))
app.close()

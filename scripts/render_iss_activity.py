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
ap.add_argument("--speed", type=float, default=1.0, help="playback speed: output fps = 30 / every * speed")
ap.add_argument("--start-s", type=float, default=0.0, help="first simulated second to render (previews)")
ap.add_argument("--end-s", type=float, default=1e9, help="last simulated second to render (previews)")
ap.add_argument("--fill", type=float, default=12000.0, help="camera-attached fill light intensity (0 = off)")
ap.add_argument("--wb", type=float, default=0.6, help="grey-world white-balance strength in post (0 = off)")
ap.add_argument("--out", type=Path, help="output mp4 (default: outputs/iss_activity/<run name>.mp4)")
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
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

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
cam.CreateFocalLengthAttr(18.0)
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
# camera eye is kept on the module centre line (route anchors projected to the shared module axis height
# z = -4.84 m): a free chase offset left the aisle (QA: 879 of 1735 frames black in the first render)
spec = json.loads(Path(REC["spec"]).read_text(encoding="utf-8"))["chain"]
CL = np.array([[a[0], a[1], -4.84] for a in spec["anchors"]])


SEG = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(CL, axis=0), axis=1))]      # arc length at each anchor


def arc_of(p):
    """Arc length of the centre-line point nearest to p."""
    best, bd = 0.0, 1e9
    for k, (a, b) in enumerate(zip(CL[:-1], CL[1:])):
        ab = b - a
        u = 0.0 if ab @ ab < 1e-9 else float(np.clip((p - a) @ ab / (ab @ ab), 0, 1))
        dd = np.linalg.norm(p - (a + u * ab))
        if dd < bd:
            best, bd = SEG[k] + u * np.sqrt(ab @ ab), dd
    return best


def at_arc(s):
    if s < 0:                     # behind the start: extend the first segment (up to 1.5 m) so the eye stays back
        d0 = (CL[1] - CL[0]) / max(np.linalg.norm(CL[1] - CL[0]), 1e-9)
        return CL[0] + max(s, -1.5) * d0
    s = float(min(s, SEG[-1]))
    k = min(int(np.searchsorted(SEG, s, side="right")) - 1, len(CL) - 2)
    u = (s - SEG[k]) / max(SEG[k + 1] - SEG[k], 1e-9)
    return CL[k] + u * (CL[k + 1] - CL[k])
# chase camera on the centre line: CAM_BACK_M of arc length behind the (smoothed) pelvis, looking along the route
# at the pelvis. Following the pelvis velocity pointed the camera at walls during re-orientation swings and the
# trailing legs filled the frame at 1.6 m (preview QA).
CAM_BACK_M = 2.3
K = 45
smooth = np.array([pel[max(0, i - K):i + K + 1].mean(axis=0) for i in range(len(pel))])
arc = np.array([arc_of(p) for p in smooth])
arc = np.maximum.accumulate(arc)                       # the camera never backs up along the route
if args.fill > 0:                                       # fill light riding with the camera (render-only)
    fl = UsdLux.SphereLight.Define(stage, "/World/Cameras/Fill")
    fl.CreateRadiusAttr(0.08)
    fl.CreateIntensityAttr(args.fill)
    fl.CreateEnableColorTemperatureAttr(True)
    fl.CreateColorTemperatureAttr(6000.0)
    fl_op = UsdGeom.Xformable(fl).AddTranslateOp()
step_at = []                                 # step number / status per frame from the segment end times
ends = [(s_.get("t_s", 1e9), s_["step"], s_["status"]) for s_ in REC["segments"]]
font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", int(22 * args.width / 1280))
small = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", int(17 * args.width / 1280))
mp4 = args.out or OUT / (args.run.stem.replace("_run", "") + ".mp4")
frames_dir = OUT / ("render_frames_" + mp4.stem)
shutil.rmtree(frames_dir, ignore_errors=True)
frames_dir.mkdir(parents=True)
n_out = 0
route = spec.get("route", "main")
ROUTE_TEXT = {"main": "Node1 > US Lab > Node2 > Columbus", "jem": "Node2 > JEM PM", "node3": "Node1 > Node3"}
cev = {s_["step"]: sum((s_.get("contacts") or {}).values()) for s_ in REC["segments"]}
for i in range(0, len(FR["t"]), args.every):
    if not args.start_s <= FR["t"][i] <= args.end_s:
        continue
    apply_frame(i)
    eye = at_arc(arc[i] - CAM_BACK_M) + np.array([0, 0, 0.10])
    look = 0.5 * (smooth[i] + at_arc(arc[i] + 0.4))          # pelvis, pulled toward the centre line ahead
    cam_op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*look), Gf.Vec3d(0, 0, 1)).GetInverse())
    if args.fill > 0:
        fl_op.Set(Gf.Vec3d(*(eye + np.array([0, 0, 0.15]))))
    app.update()
    rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0, pause_timeline=True)
    a = np.asarray(rgb.get_data())[:, :, :3].astype(np.float32)
    if args.wb > 0:                                    # partial grey-world white balance (the 4800 K lights read brown)
        m = a.reshape(-1, 3).mean(axis=0) + 1e-6
        a = a * (1 - args.wb + args.wb * m.mean() / m)
    img = Image.fromarray(np.clip(a, 0, 255).astype(np.uint8)).convert("RGBA")
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    t = FR["t"][i]
    done = [e for e in ends if e[0] <= t]
    nxt = next((e for e in ends if e[0] > t), None)
    status = f"step {nxt[1]} of {REC.get('total_steps')}" if nxt else (f"step {done[-1][1]}: {done[-1][2]}" if done else "")
    cur = nxt[1] if nxt else (done[-1][1] if done else None)
    g_txt = f"gravity {REC.get('gravity_m_s2', 0.0):g} m/s²" if REC.get("gravity_m_s2") else "microgravity (0 g)"
    lines = [(f"G1 · PhysX · {g_txt} · hand-over-hand {ROUTE_TEXT.get(route, route)} · closed-loop cuRobo",
              font),
             (f"t = {t:5.1f} s   {status}   captures so far: {sum(e[2] == 'CAPTURED' for e in done)}"
              + ("" if REC.get("contacts_filtered") or cur not in cev else f"   contact events this step: {cev[cur]}"),
              font),
             ("scripted grasps (fixed joints) · PLACEHOLDER handholds outside the S1 rail and hatch rims · "
              + ("G1-ISS contacts filtered" if REC.get("contacts_filtered") else "G1-ISS contacts ON (PhysX)")
              + (f" · {args.speed:g}x speed" if args.speed != 1 else ""), small)]
    y = int(18 * args.width / 1280)
    dr.rectangle([0, 0, img.size[0], y + sum(int(f.size * 1.4) for _, f in lines) + 6], fill=(0, 0, 0, 110))
    for text, f in lines:
        dr.text((21, y + 1), text, font=f, fill=(0, 0, 0, 170))
        dr.text((20, y), text, font=f, fill=(255, 255, 255, 235))
        y += int(f.size * 1.4)
    Image.alpha_composite(img, ov).convert("RGB").save(frames_dir / f"f{n_out:05d}.png")
    n_out += 1
fps = max(1, round(30 / args.every * args.speed))
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", str(frames_dir / "f%05d.png"),
                "-c:v", "libx264", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)], check=True)
print(json.dumps({"video": str(mp4), "frames": n_out, "fps": fps, "run": str(args.run)}))
app.close()

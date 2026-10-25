"""Lunar closing shot: G1 walks on a regolith patch, leaves bootprints, kicks up ballistic dust (visual).

C:\\isaacsim\\python.bat scripts/render_lunar_footprints.py [--rec outputs/lunar_v0/scene_walk_rec.json]
    [--speed 1.0] [--subframes 12] [--out outputs/lunar_v0/lunar_footprints.mp4]
Robot motion: a recorded Isaac Lab run (lunar_walk_eval.py --record) of the P3 lunar policy (1.62 m/s^2, rigid
ground), replayed on Isaac Lab's G1 USD (physics disabled). Everything below is render-only and labelled so:
- bootprints: at each recorded touchdown, a foot-shaped depression (FOOT_L x FOOT_W, DEPTH, with a raised rim and
  cross treads) is stamped into a 2 cm heightfield along the path. Shape and depth are illustrative, not computed
  from soil mechanics;
- dust: at each touchdown and lift-off, DUST_N grains leave the sole with random speeds and fly drag-free
  parabolas under 1.62 m/s^2 (no atmosphere), then rest on the ground. Launch speeds are assumed;
- sky: black, low Sun (DistantLight, 8 deg elevation), Earth (NASA Blue Marble NG) at about 1.9 deg apparent size.
Output: outputs/lunar_v0/lunar_footprints.mp4 (gitignored).

--pov: first-person finale for the showcase (docs/SHOWCASE_STORYBOARD.md shot 10) instead of the chase shot. The
camera rides on head_link (eye at the front of the head mesh); its orientation is the head frame low-passed with a
zero-phase Gaussian (-3 dB at --lp-hz) so gait bob stays and jitter goes, pitched --pitch0 deg down, easing to
--pitch1 over --tilt-s screen seconds before the last touchdown (first touchdown at or after --last-td). When that
foot lifts, the camera keeps riding on the head but turns to hold the fresh print in view for --hold-s screen
seconds while the replay runs on. The Sun (8 deg up, from the left and behind) and Earth (about 4 deg up, ahead
and slightly right, 1.9 deg across) are placed relative to the walking direction at the last step. No overlay is
burned in; a <out>_frames.json sidecar (t, phase per frame) is written for the editor.
"""
import argparse
import json
import math
import shutil
import subprocess
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--rec", type=Path, default=Path("outputs/lunar_v0/scene_walk_rec.json"))
ap.add_argument("--speed", type=float, default=1.0)
ap.add_argument("--subframes", type=int, default=12)
ap.add_argument("--fps", type=int, default=25)
ap.add_argument("--width", type=int, default=1280)
ap.add_argument("--height", type=int, default=720)
ap.add_argument("--max-s", type=float, default=1e9, help="stop after this many seconds of motion (previews)")
ap.add_argument("--outro-s", type=float, default=5.0, help="closing footprint shot length")
ap.add_argument("--out", type=Path, default=Path("outputs/lunar_v0/lunar_footprints.mp4"))
ap.add_argument("--pov", action="store_true", help="first-person finale (see docstring)")
ap.add_argument("--start", type=float, default=15.5, help="--pov: first replayed second")
ap.add_argument("--last-td", type=float, default=17.9, help="--pov: the last step is the first touchdown after this")
ap.add_argument("--pitch0", type=float, default=15.0)
ap.add_argument("--pitch1", type=float, default=60.0, help="35 (brief) keeps the feet out of a 70 deg view")
ap.add_argument("--tilt-s", type=float, default=3.0)
ap.add_argument("--hold-s", type=float, default=2.0)
ap.add_argument("--fov", type=float, default=70.0, help="--pov: horizontal field of view, deg")
ap.add_argument("--lp-hz", type=float, default=1.5)
ap.add_argument("--sun", type=float, default=4500.0, help="Sun intensity (lower keeps the white shell from clipping)")
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": args.width, "height": args.height})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
G_MOON = 1.62
FOOT_L, FOOT_W, DEPTH, RIM = 0.24, 0.10, 0.022, 0.005
FOOT_FWD = 0.035     # print centre ahead of the ankle-roll link origin (the ankle sits toward the heel)
DUST_N, CELL = 40, 0.015
EARTH_TEX = ROOT / "assets" / "raw" / "earth" / "world.topo.200407.3x5400x2700.jpg"
rng = np.random.default_rng(7)

R = json.loads((ROOT / args.rec).read_text(encoding="utf-8"))["per_seed"][0]["record"]
T = np.asarray(R["t"])
POS = np.asarray(R["pos"])                      # (steps, bodies, 3)
QUAT = np.asarray(R["quat_wxyz"])
CON = np.asarray(R["contact"])                  # (steps, 2)
names = R["body_names"]
feet = [names.index(f) for f in R["feet"]]
n_steps = len(T)
# the recorded ground is at z = 0 in the Isaac Lab env frame; shift the whole run so the path starts at the origin
off = np.array([POS[0, 0, 0], POS[0, 0, 1], 0.0])
POS = POS - off

# ---------------------------------------------------------------- touchdowns / lift-offs
events = []          # (t, foot index, kind, pos, yaw)
for k in range(2):
    c = CON[:, k]
    for i in np.flatnonzero(np.diff(c) != 0) + 1:
        q = QUAT[i, feet[k]]
        w, x, y, z = q
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        events.append((float(T[i]), k, "down" if c[i] else "up", POS[i, feet[k]].copy(), yaw))
events.sort(key=lambda e: e[0])

# ---------------------------------------------------------------- stage
s = carb.settings.get_settings()
for key, v in (("/rtx/rendermode", "PathTracing"), ("/rtx/pathtracing/spp", 1),
               ("/rtx/pathtracing/totalSpp", args.subframes), ("/rtx/pathtracing/optixDenoiser/enabled", True),
               ("/rtx/post/motionblur/enabled", False), ("/rtx/post/histogram/enabled", False)):
    s.set(key, v)
ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)


def material(path, rgb, rough=0.95, tex=None, primvar_color=False):
    m = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.0)
    if tex:
        tx = UsdShade.Shader.Define(stage, path + "/T")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(tex))
        rd = UsdShade.Shader.Define(stage, path + "/R")
        rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    elif primvar_color:
        pv = UsdShade.Shader.Define(stage, path + "/C")
        pv.CreateIdAttr("UsdPrimvarReader_float3")
        pv.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(pv.ConnectableAPI(), "result")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return m


regolith = material("/World/Looks/Regolith", (0.42, 0.41, 0.39))
dustm = material("/World/Looks/Dust", (0.40, 0.39, 0.37))

# heightfield over the walked corridor (2 cm cells), a coarse plane around it
path_xy = POS[:, 0, :2]
lo, hi = path_xy.min(0) - 1.0, path_xy.max(0) + 1.0
nx, ny = int((hi[0] - lo[0]) / CELL) + 1, int((hi[1] - lo[1]) / CELL) + 1
gx, gy = np.meshgrid(lo[0] + CELL * np.arange(nx), lo[1] + CELL * np.arange(ny), indexing="xy")
# scattered shallow dimples only (soft Gaussian bowls): any fine noise, even 0.2 mm, and a 6 cm undulation read as
# water ripples under the 8 deg Sun, and sharp-rimmed bowls read as coins; all illustrative
H = np.zeros((ny, nx))
for _ in range(int((hi[0] - lo[0]) * (hi[1] - lo[1]) * 0.3)):
    cx_, cy_, rad = rng.uniform(lo[0], hi[0]), rng.uniform(lo[1], hi[1]), rng.uniform(0.05, 0.3)
    H -= 0.04 * rad * np.exp(-((gx - cx_) ** 2 + (gy - cy_) ** 2) / (2 * (0.5 * rad) ** 2))
edge = np.minimum.reduce([gx - lo[0], hi[0] - gx, gy - lo[1], hi[1] - gy])
FADE_E = np.clip(edge / 0.5, 0.0, 1.0)
H = H * FADE_E - 0.003 * (1 - FADE_E)                       # meet the plane (z = -3 mm) without a step or seam
idx = np.arange(nx * ny).reshape(ny, nx)
quads = np.stack([idx[:-1, :-1], idx[:-1, 1:], idx[1:, 1:], idx[1:, :-1]], -1).reshape(-1, 4)
field = UsdGeom.Mesh.Define(stage, "/World/Ground/Field")
field.CreateFaceVertexCountsAttr([4] * len(quads))
field.CreateFaceVertexIndicesAttr(quads.ravel().tolist())
field.CreateSubdivisionSchemeAttr("none")
field.CreateNormalsAttr()
field.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
pts_attr = field.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.stack([gx, gy, H], -1).reshape(-1, 3).astype(np.float32)))
# brightness variation of the corridor (albedo only, no relief, so it cannot shade into ripples): two scales of
# smoothed noise, +-6 % around the regolith grey, read through displayColor; illustrative
from scipy.ndimage import gaussian_filter  # noqa: E402
alb = gaussian_filter(rng.standard_normal((ny, nx)), 1.0) + 0.6 * gaussian_filter(rng.standard_normal((ny, nx)), 3.0)
alb = 1.0 + 0.06 * alb / max(alb.std(), 1e-9) * FADE_E     # fades to the plane's grey at the corridor edge
field.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(Vt.Vec3fArray.FromNumpy(
    (alb.reshape(-1, 1) * np.array([0.42, 0.41, 0.39])).astype(np.float32)))
regolith_v = material("/World/Looks/RegolithVar", None, primvar_color=True)
UsdShade.MaterialBindingAPI.Apply(field.GetPrim()).Bind(regolith_v)
plane = UsdGeom.Mesh.Define(stage, "/World/Ground/Plane")
plane.CreatePointsAttr([(-200, -200, -0.003), (200, -200, -0.003), (200, 200, -0.003), (-200, 200, -0.003)])
plane.CreateFaceVertexCountsAttr([4])
plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(regolith)


def set_field():
    pts = np.stack([gx, gy, H], -1).reshape(-1, 3).astype(np.float32)
    pts_attr.Set(Vt.Vec3fArray.FromNumpy(pts))
    dy_, dx_ = np.gradient(H, CELL)                            # smooth vertex normals: no grid faceting up close
    n = np.stack([-dx_, -dy_, np.ones_like(H)], -1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    field.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(n.reshape(-1, 3).astype(np.float32)))


def stamp(p, yaw):
    """Bootprint at foot position p, heading yaw: depression with cross treads and a raised rim (illustrative).
    Each print gets a small random yaw, depth and tread phase so the track does not read as stamped copies."""
    yaw += math.radians(rng.uniform(-4.0, 4.0))
    depth, phase = DEPTH * rng.uniform(0.85, 1.15), rng.uniform(0.0, 2 * math.pi)
    c, s_ = math.cos(yaw), math.sin(yaw)
    dx, dy = gx - (p[0] + FOOT_FWD * c), gy - (p[1] + FOOT_FWD * s_)
    u, v = c * dx + s_ * dy, -s_ * dx + c * dy                  # along / across the foot
    r = ((np.abs(u) / (FOOT_L / 2)) ** 4 + (np.abs(v) / (FOOT_W / 2)) ** 4) ** 0.25   # rounded-rectangle outline
    wall = np.clip((1.1 - r) / 0.25, 0.0, 1.0)
    wall = wall * wall * (3 - 2 * wall)                      # smooth wall, flat floor
    tread = 0.004 * (0.5 + 0.5 * np.cos(u * 2 * math.pi / 0.05 + phase)) * np.clip((wall - 0.5) * 2, 0, 1)
    rim = RIM * np.exp(-((r - 1.3) / 0.22) ** 2) * (r > 1.0)
    H[:] = np.minimum(H + rim, -(depth * wall - tread) + H * (1 - wall))


# Robot: Isaac Lab's G1 (same asset as the trained policy), physics off, bodies posed from the record
url = f"{get_assets_root_path()}/Isaac/IsaacLab/Robots/Unitree/G1/g1_minimal.usd"
g1 = stage.DefinePrim("/World/G1", "Xform")
g1.GetReferences().AddReference(url)
for _ in range(30):
    app.update()
for p in list(Usd.PrimRange(g1)):
    if p.IsInstance():
        p.SetInstanceable(False)
for p in list(Usd.PrimRange(g1)):
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
    if p.IsA(UsdPhysics.Joint):
        UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        PhysxSchema.PhysxArticulationAPI.Apply(p).CreateArticulationEnabledAttr(False)
body_ops = []
for nme in names:
    prim = stage.GetPrimAtPath(f"/World/G1/{nme}")
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    body_ops.append((x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)))

# dust grains (points), sky, Sun, Earth
dust = UsdGeom.Points.Define(stage, "/World/Dust")
UsdShade.MaterialBindingAPI.Apply(dust.GetPrim()).Bind(dustm)
grains = []                                                    # [x, y, z, vx, vy, vz, t0, resting]
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(args.sun)
sun.CreateAngleAttr(0.53)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(82.0, 0.0, 35.0))   # ~8 deg above the horizon
fill = UsdLux.DomeLight.Define(stage, "/World/Earthshine")
fill.CreateIntensityAttr(4.0)
fill.CreateColorAttr(Gf.Vec3f(0.6, 0.7, 1.0))
nu, nv = 64, 32
uu, vv = np.meshgrid(np.linspace(0, 1, nu + 1), np.linspace(0, 1, nv + 1))
lon, lat = (uu - 0.5) * 2 * math.pi, (0.5 - vv) * math.pi
ep = np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], -1).reshape(-1, 3) * 15.0
ef = []
for j in range(nv):
    for i in range(nu):
        a = j * (nu + 1) + i
        ef += [a, a + nu + 1, a + nu + 2, a + 1]
earth = UsdGeom.Mesh.Define(stage, "/World/Earth")
earth.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(ep.astype(np.float32)))
earth.CreateFaceVertexCountsAttr([4] * (nu * nv))
earth.CreateFaceVertexIndicesAttr(ef)
st_ = UsdGeom.PrimvarsAPI(earth).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
st_.Set(Vt.Vec2fArray.FromNumpy(np.stack([uu.ravel(), 1 - vv.ravel()], -1).astype(np.float32)[np.asarray(ef)]))
UsdShade.MaterialBindingAPI.Apply(earth.GetPrim()).Bind(material("/World/Looks/EarthMat", (1, 1, 1), 1.0, EARTH_TEX))
mid = POS[n_steps // 2, 0]
UsdGeom.Xformable(earth).AddTranslateOp().Set(Gf.Vec3d(mid[0] + 600.0, mid[1] + 250.0, 260.0))   # ~1.3 deg up-ish

cam = UsdGeom.Camera.Define(stage, "/World/Cam")
cam.CreateFocalLengthAttr(24.0)
cam.CreateHorizontalApertureAttr(36.0)
cam.CreateVerticalApertureAttr(36.0 * args.height / args.width)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.03, 2000.0))
cam_op = UsdGeom.Xformable(cam).AddTransformOp()
set_field()
for _ in range(60):
    app.update()
rp = rep.create.render_product(str(cam.GetPath()), (args.width, args.height))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach(rp)
rep.orchestrator.set_capture_on_play(False)

font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", int(22 * args.width / 1280))
small = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", int(16 * args.width / 1280))
fdir = ROOT / "outputs" / "lunar_v0" / ("render_frames_" + (args.out.stem if args.pov else "footprints"))
shutil.rmtree(fdir, ignore_errors=True)
fdir.mkdir(parents=True)


def look(eye, target):
    cam_op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse())


def frame_out(k, lines):
    rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0, pause_timeline=True)
    img = Image.fromarray(np.asarray(rgb.get_data())[:, :, :3].astype(np.uint8)).convert("RGBA")
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    y = 16
    for text, f in lines:
        dr.text((20, y), text, font=f, fill=(255, 255, 255, 220))
        y += int(f.size * 1.4)
    Image.alpha_composite(img, ov).convert("RGB").save(fdir / f"f{k:05d}.png")


def launch(te, kind, p, yaw):
    """Bootprint (touchdown) and dust grains for one contact event."""
    if kind == "down":
        stamp(p, yaw)
    sole = np.array([p[0], p[1], 0.005])
    for _ in range(DUST_N if kind == "down" else DUST_N // 2):
        a = rng.uniform(0, 2 * math.pi)
        sp = rng.uniform(0.15, 0.7)
        grains.append([*sole, sp * math.cos(a) + 0.2 * math.cos(yaw), sp * math.sin(a) + 0.2 * math.sin(yaw),
                       rng.uniform(0.2, 0.9), te, 0.0])


def pose_scene(t, i):
    set_field()
    if grains:                                                 # drag-free ballistic flight at lunar gravity
        g = np.asarray(grains)
        tau = np.clip(t - g[:, 6], 0, None)
        tf = 2 * g[:, 5] / G_MOON                              # time of flight back to z = 0
        tt = np.minimum(tau, tf)
        gp = np.stack([g[:, 0] + g[:, 3] * tt, g[:, 1] + g[:, 4] * tt,
                       np.maximum(0.002, g[:, 2] + g[:, 5] * tt - 0.5 * G_MOON * tt ** 2)], -1)
        dust.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(gp.astype(np.float32)))
        dust.GetWidthsAttr().Set(Vt.FloatArray([0.0025] * len(gp)))
    for b, (to, oo) in enumerate(body_ops):
        to.Set(Gf.Vec3d(*POS[i, b]))
        oo.Set(Gf.Quatd(*QUAT[i, b]))


if args.pov:
    from scipy.ndimage import gaussian_filter1d
    from scipy.spatial.transform import Rotation
    hb = names.index("head_link")
    for b, (to, oo) in enumerate(body_ops):                    # pose frame 0 to measure the head mesh
        to.Set(Gf.Vec3d(*POS[0, b]))
        oo.Set(Gf.Quatd(*QUAT[0, b]))
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeUntransformedBound(
        stage.GetPrimAtPath("/World/G1/head_link")).ComputeAlignedRange()
    lo_b, hi_b = np.array(box.GetMin()), np.array(box.GetMax())
    # head frame; 0.18 m ahead of the face so the chest does not fill the frame when looking down at the feet
    EYE = np.array([hi_b[0] + 0.18, 0.5 * (lo_b[1] + hi_b[1]), lo_b[2] + 0.6 * (hi_b[2] - lo_b[2])])
    rot = Rotation.from_quat(QUAT[:, hb][:, [1, 2, 3, 0]])     # wxyz -> xyzw
    sig = 0.1325 / args.lp_hz / float(R["dt"])                 # Gaussian sigma (samples) with -3 dB at lp_hz
    fwd = gaussian_filter1d(rot.apply([1.0, 0, 0]), sig, axis=0)
    upv = gaussian_filter1d(rot.apply([0, 0, 1.0]), sig, axis=0)
    eye_w = POS[:, hb] + rot.apply(EYE)
    last = next(e for e in events if e[2] == "down" and e[0] >= args.last_td)
    lift = next(e for e in events if e[2] == "up" and e[1] == last[1] and e[0] > last[0])
    il = int(np.searchsorted(T, last[0]))
    hd = fwd[il, :2] / np.linalg.norm(fwd[il, :2])                # walking heading at the last step
    def az(deg, el, dist):
        c_, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        h_ = np.array([c_ * hd[0] - s_ * hd[1], s_ * hd[0] + c_ * hd[1]])
        return dist * np.array([h_[0] * math.cos(math.radians(el)), h_[1] * math.cos(math.radians(el)),
                                math.sin(math.radians(el))])
    sun_x = UsdGeom.Xformable(sun)
    sun_x.ClearXformOpOrder()
    sun_x.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(0, 0, 0), Gf.Vec3d(*(-az(120.0, 8.0, 1.0))),
                                                       Gf.Vec3d(0, 0, 1)).GetInverse())
    e_x = UsdGeom.Xformable(earth)
    e_x.ClearXformOpOrder()
    e_x.AddTranslateOp().Set(Gf.Vec3d(*(eye_w[il] + az(-10.0, 4.0, 900.0))))   # r 15 m at 900 m: 1.9 deg
    cam.CreateFocalLengthAttr(18.0 / math.tan(math.radians(args.fov / 2)))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.03, 2000.0))
    dt_out = args.speed / args.fps
    t_tilt0 = last[0] - args.tilt_s * args.speed
    for e in [e for e in events if e[0] < args.start]:         # prints already on the ground when the shot opens
        if e[2] == "down":
            stamp(e[3], e[4])
    ev_i = next(k_ for k_, e in enumerate(events) if e[0] >= args.start)
    t, k, held, side = args.start, 0, None, []
    t_end = lift[0] + args.hold_s * args.speed
    while t <= t_end:
        i = min(int(np.searchsorted(T, t)), n_steps - 1)
        while ev_i < len(events) and events[ev_i][0] <= t:
            launch(*[events[ev_i][j] for j in (0, 2, 3, 4)])
            ev_i += 1
        pose_scene(t, i)
        a = float(np.clip((t - t_tilt0) / max(last[0] - t_tilt0, 1e-9), 0, 1))
        pitch = math.radians(args.pitch0 + (args.pitch1 - args.pitch0) * (0.5 - 0.5 * math.cos(math.pi * a)))
        f = fwd[i] / np.linalg.norm(fwd[i])
        rgt = np.cross(f, upv[i])
        rgt /= np.linalg.norm(rgt)
        u = np.cross(rgt, f)
        if t < lift[0]:
            held = eye_w[i] + math.cos(pitch) * f - math.sin(pitch) * u
        else:                                                  # keep the fresh print in view
            pr = np.array([last[3][0] + FOOT_FWD * math.cos(last[4]), last[3][1] + FOOT_FWD * math.sin(last[4]), 0.0])
            b_ = min(1.0, (t - lift[0]) / (0.5 * args.speed))
            held = (1 - b_) * held + b_ * pr if b_ < 1 else pr
        target = held
        phase = "hold" if t >= lift[0] else ("tilt" if a > 0 else "walk")
        cam_op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye_w[i]), Gf.Vec3d(*target), Gf.Vec3d(*u)).GetInverse())
        app.update()
        rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0, pause_timeline=True)
        Image.fromarray(np.asarray(rgb.get_data())[:, :, :3].astype(np.uint8)).save(fdir / f"f{k:05d}.png")
        side.append({"t": round(t, 4), "phase": phase})
        k += 1
        t += dt_out
    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps), "-i", str(fdir / "f%05d.png"),
                    "-c:v", "libx264", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)],
                   check=True)
    out.with_name(out.stem + "_frames.json").write_text(json.dumps(
        {"rec": str(args.rec), "speed": args.speed, "fps": args.fps, "last_touchdown_t": last[0],
         "lift_t": lift[0], "frames": side}), encoding="utf-8")
    print(json.dumps({"video": str(out), "frames": k, "last_touchdown_t": last[0], "lift_t": lift[0]}))
    app.close()
    raise SystemExit

caption = [("Unitree G1 on the Moon (1.62 m/s²) · walking policy trained in Isaac Lab, replayed", font),
           ("bootprints and dust are visual effects: illustrative print shape; grains fly drag-free parabolas at lunar "
            "gravity", small)]
dt_out = args.speed / args.fps
t_end = min(float(T[-1]), args.max_s)
ev_i, k = 0, 0
t = 0.0
smooth = np.array([POS[max(0, i - 25):i + 26, 0].mean(0) for i in range(n_steps)])
while t <= t_end:
    i = min(int(np.searchsorted(T, t)), n_steps - 1)
    while ev_i < len(events) and events[ev_i][0] <= t:      # footprints and dust launched since the last frame
        launch(*[events[ev_i][j] for j in (0, 2, 3, 4)])
        ev_i += 1
    pose_scene(t, i)
    c = smooth[i]
    look(c + np.array([-1.6, -2.0, 0.75]), c + np.array([0.3, 0.0, 0.25]))
    app.update()
    frame_out(k, caption)
    k += 1
    t += dt_out
# closing shot: the robot walks off, the camera settles low over the bootprints
td = [e for e in events if e[2] == "down" and e[0] <= t_end]
focus = np.mean([e[3] for e in td[len(td) // 3: len(td) // 3 + 4]], axis=0)
n_out = int(args.outro_s * args.fps)
for j in range(n_out):
    a = 0.5 - 0.5 * math.cos(math.pi * j / max(n_out - 1, 1))
    eye = (1 - a) * (focus + np.array([-2.0, -2.4, 1.2])) + a * (focus + np.array([-0.55, -0.45, 0.42]))
    look(eye, focus + np.array([0.15, 0.0, 0.0]))
    app.update()
    frame_out(k, [("Apollo 11 left the first bootprints in 1969. Here: a simulated G1 (visual effect)", font)])
    k += 1
args.out.parent.mkdir(parents=True, exist_ok=True)
out = ROOT / args.out
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps), "-i", str(fdir / "f%05d.png"),
                "-c:v", "libx264", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)], check=True)
print(json.dumps({"video": str(out), "frames": k, "touchdowns": len(td)}))
app.close()

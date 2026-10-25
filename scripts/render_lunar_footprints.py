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


def material(path, rgb, rough=0.95, tex=None):
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
# faint, isotropic undulation (Gaussian-filtered noise, 1.5 mm rms) fading to 0 at the corridor edge so the
# heightfield meets the flat plane without a step (a 1-D filter in the first preview read as dunes)
H = rng.standard_normal((ny, nx))
ky, kx = np.meshgrid(np.fft.fftfreq(ny, CELL), np.fft.fftfreq(nx, CELL), indexing="ij")
H = np.real(np.fft.ifft2(np.fft.fft2(H) * np.exp(-(kx ** 2 + ky ** 2) * (2 * math.pi * 0.06) ** 2 / 2)))
H *= 0.001 / max(H.std(), 1e-9)
edge = np.minimum.reduce([gx - lo[0], hi[0] - gx, gy - lo[1], hi[1] - gy])
H *= np.clip(edge / 0.5, 0.0, 1.0)
idx = np.arange(nx * ny).reshape(ny, nx)
quads = np.stack([idx[:-1, :-1], idx[:-1, 1:], idx[1:, 1:], idx[1:, :-1]], -1).reshape(-1, 4)
field = UsdGeom.Mesh.Define(stage, "/World/Ground/Field")
field.CreateFaceVertexCountsAttr([4] * len(quads))
field.CreateFaceVertexIndicesAttr(quads.ravel().tolist())
field.CreateSubdivisionSchemeAttr("none")
field.CreateNormalsAttr()
field.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
pts_attr = field.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.stack([gx, gy, H], -1).reshape(-1, 3).astype(np.float32)))
UsdShade.MaterialBindingAPI.Apply(field.GetPrim()).Bind(regolith)
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
    """Bootprint at foot position p, heading yaw: depression with cross treads and a raised rim (illustrative)."""
    c, s_ = math.cos(yaw), math.sin(yaw)
    dx, dy = gx - (p[0] + FOOT_FWD * c), gy - (p[1] + FOOT_FWD * s_)
    u, v = c * dx + s_ * dy, -s_ * dx + c * dy                  # along / across the foot
    r = ((np.abs(u) / (FOOT_L / 2)) ** 4 + (np.abs(v) / (FOOT_W / 2)) ** 4) ** 0.25   # rounded-rectangle outline
    wall = np.clip((1.1 - r) / 0.25, 0.0, 1.0)
    wall = wall * wall * (3 - 2 * wall)                      # smooth wall, flat floor
    tread = 0.005 * (0.5 + 0.5 * np.cos(u * 2 * math.pi / 0.05)) * np.clip((wall - 0.5) * 2, 0, 1)
    rim = RIM * np.exp(-((r - 1.25) / 0.15) ** 2) * (r > 1.0)
    H[:] = np.minimum(H + rim, -(DEPTH * wall - tread) + H * (1 - wall))


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
sun.CreateIntensityAttr(4500.0)
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
fdir = ROOT / "outputs" / "lunar_v0" / "render_frames_footprints"
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
        te, fk, kind, p, yaw = events[ev_i]
        if kind == "down":
            stamp(p, yaw)
        sole = np.array([p[0], p[1], 0.005])
        for _ in range(DUST_N if kind == "down" else DUST_N // 2):
            a = rng.uniform(0, 2 * math.pi)
            sp = rng.uniform(0.15, 0.7)
            grains.append([*sole, sp * math.cos(a) + 0.2 * math.cos(yaw), sp * math.sin(a) + 0.2 * math.sin(yaw),
                           rng.uniform(0.2, 0.9), te, 0.0])
        ev_i += 1
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

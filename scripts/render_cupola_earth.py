"""Earth from the Cupola position with a time-compressed orbital day/night cycle (visual only; no orbital mechanics).

C:\\isaacsim\\python.bat scripts/render_cupola_earth.py [--seconds 30] [--fps 20] [--subframes 16]
Opens scenes/iss_full_v0.usda (visual layer) and adds, in memory only: a UV sphere textured with NASA Blue Marble
Next Generation (assets/raw/earth, see its PROVENANCE.json) and a distant "Sun" light. Geometry is scaled 1:1000
(Earth radius 6371 m, 400 m below the station) so the angular size of Earth matches a ~400 km orbit. One orbit
(ORBIT_S = 5550 s, ~92.5 min) is compressed into --seconds: the ground track moves under the Cupola (Earth turns
about the orbit normal) and the Sun direction turns at the same rate in the station frame, so the clip passes
through day, sunset and night. The station's own lights stay on. Output: outputs/iss_demos/cupola_earth.mp4.
--phase0/--phase1 render only that fraction of the orbit (e.g. the day pass into sunset) over --seconds; --clean
drops the burned-in text and writes <out>_frames.json (orbit minutes per frame, speed factor) for an editor.
"""
import argparse
import json
import shutil
import subprocess
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--seconds", type=float, default=30.0)
ap.add_argument("--fps", type=int, default=20)
ap.add_argument("--subframes", type=int, default=16)
ap.add_argument("--debug-far", action="store_true", help="diagnostic: camera far outside, looking at Earth")
ap.add_argument("--debug-hide", default="", help="diagnostic: comma-separated prim paths made invisible")
ap.add_argument("--width", type=int, default=1280)
ap.add_argument("--height", type=int, default=720)
ap.add_argument("--phase0", type=float, default=None, help="orbit fraction at the first frame (default PHASE0)")
ap.add_argument("--phase1", type=float, default=None, help="orbit fraction at the last frame (default +1 orbit)")
ap.add_argument("--clean", action="store_true")
ap.add_argument("--sun", type=float, default=1200.0, help="Sun intensity")
ap.add_argument("--out", type=Path, default=None)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": args.width, "height": args.height})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "iss_demos"
TEX = ROOT / "assets" / "raw" / "earth" / "world.topo.200407.3x5400x2700.jpg"
SCALE = 1e-3                      # 1:1000
R_EARTH, ALT = 6371e3 * SCALE, 400e3 * SCALE
ORBIT_S = 5550.0
PHASE0 = 0.55                     # start shortly before orbital sunrise (the night pass was ~half of the clip)

s = carb.settings.get_settings()
for k, v in (("/rtx/rendermode", "PathTracing"), ("/rtx/pathtracing/spp", 1),
             ("/rtx/pathtracing/totalSpp", args.subframes), ("/rtx/pathtracing/optixDenoiser/enabled", True),
             ("/rtx/post/motionblur/enabled", False), ("/rtx/post/histogram/enabled", False)):
    s.set(k, v)
ctx = omni.usd.get_context()
ctx.open_stage(str(ROOT / "scenes" / "iss_full_v0.usda"))
stage = ctx.get_stage()
stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection("visual")
rep_cfg = json.loads((ROOT / "outputs" / "iss_full_v0" / "build_report.json").read_text(encoding="utf-8"))
cup = rep_cfg["cameras"]["Cupola_Wide"]
eye = np.asarray(cup["eye"], float)
centre = eye + np.array([0.0, 0.0, -(ALT + R_EARTH)])          # nadir = scene -z below the Cupola

# UV sphere (lat-long st) so the equirectangular texture maps directly
nu, nv = 256, 128
uu, vv = np.meshgrid(np.linspace(0, 1, nu + 1), np.linspace(0, 1, nv + 1))
lon, lat = (uu - 0.5) * 2 * np.pi, (0.5 - vv) * np.pi
pts = np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], -1).reshape(-1, 3) * R_EARTH
faces = []
for j in range(nv):
    for i in range(nu):
        a = j * (nu + 1) + i
        faces += [a, a + nu + 1, a + nu + 2, a + 1]
earth = UsdGeom.Mesh.Define(stage, "/World/Orbit/Earth")
earth.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
earth.CreateFaceVertexCountsAttr([4] * (nu * nv))
earth.CreateFaceVertexIndicesAttr(faces)
earth.CreateSubdivisionSchemeAttr("none")
earth.CreateDoubleSidedAttr(True)
earth.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy((pts / R_EARTH).astype(np.float32)[np.asarray(faces)]))
earth.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
st_v = np.stack([uu.ravel(), 1 - vv.ravel()], -1).astype(np.float32)
st = UsdGeom.PrimvarsAPI(earth).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
st.Set(Vt.Vec2fArray.FromNumpy(st_v[np.asarray(faces)]))     # faceVarying: one st per face corner
mat = UsdShade.Material.Define(stage, "/World/Orbit/EarthMat")
sh = UsdShade.Shader.Define(stage, "/World/Orbit/EarthMat/Surface")
sh.CreateIdAttr("UsdPreviewSurface")
sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.0)          # no specular sheen: the oceans read white otherwise
tx = UsdShade.Shader.Define(stage, "/World/Orbit/EarthMat/Tex")
tx.CreateIdAttr("UsdUVTexture")
tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(TEX))
rd = UsdShade.Shader.Define(stage, "/World/Orbit/EarthMat/St")
rd.CreateIdAttr("UsdPrimvarReader_float2")
rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(earth.GetPrim()).Bind(mat)
e_x = UsdGeom.Xformable(earth)
e_t, e_r = e_x.AddTranslateOp(), e_x.AddRotateXYZOp()
e_t.Set(Gf.Vec3d(*centre))
sun = UsdLux.DistantLight.Define(stage, "/World/Orbit/Sun")
sun.CreateIntensityAttr(args.sun)
sun.CreateAngleAttr(0.53)
sun_r = UsdGeom.Xformable(sun).AddRotateXYZOp()
night = UsdLux.DomeLight.Define(stage, "/World/Orbit/NightFill")   # faint fill so the night side is not pure black
night.CreateIntensityAttr(25.0)
night.CreateColorAttr(Gf.Vec3f(0.55, 0.65, 1.0))

# The NASA Cupola mesh closes its windows with opaque panes; removing them in memory (glass + nearby faces, as holes,
# cut-out material or deleted faces) still left the view blocked, so the Cupola mesh is hidden for this shot: the
# camera sits at the Cupola position with nothing between it and Earth (stated on the overlay)
UsdGeom.Imageable(stage.GetPrimAtPath("/World/Environment/ISS/Visual/Cupola")).MakeInvisible()
for hp in filter(None, args.debug_hide.split(",")):
    UsdGeom.Imageable(stage.GetPrimAtPath(hp)).MakeInvisible()

cam = UsdGeom.Camera.Define(stage, "/World/Cameras/CupolaEarth")
cam.CreateFocalLengthAttr(12.0)
cam.CreateHorizontalApertureAttr(36.0)
cam.CreateVerticalApertureAttr(36.0 * args.height / args.width)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 2.0 * (ALT + 2 * R_EARTH)))
look = eye + np.array([0.55, 0.0, -1.0])                      # oblique: limb near the top of the frame
if args.debug_far:
    look = centre
    eye = centre + np.array([0.0, -3.0 * R_EARTH, 0.5 * R_EARTH])
UsdGeom.Xformable(cam).AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*look),
                                                                      Gf.Vec3d(1, 0, 0)).GetInverse())
for _ in range(60):
    app.update()
rp = rep.create.render_product(str(cam.GetPath()), (args.width, args.height))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach(rp)
rep.orchestrator.set_capture_on_play(False)
font = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", int(22 * args.width / 1280))
small = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", int(17 * args.width / 1280))
fdir = OUT / ("render_frames_" + (args.out.stem if args.out else "cupola_earth"))
shutil.rmtree(fdir, ignore_errors=True)
fdir.mkdir(parents=True)
n = int(args.seconds * args.fps)
p0 = PHASE0 if args.phase0 is None else args.phase0
span = 1.0 if args.phase1 is None else args.phase1 - p0
side = []
for k in range(n):
    phase = (p0 + span * k / n) % 1.0                          # fraction of one orbit
    ang = 360.0 * phase
    e_r.Set(Gf.Vec3f(90.0, ang, 0.0))       # equator under the station (sphere poles on +-z rotated to +-y = orbit
                                             # normal); the ground track moves under the Cupola
    sun_r.Set(Gf.Vec3f(0.0, 90.0 + ang, 0.0))                 # Sun turns at the orbital rate in the station frame
    app.update()
    rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0, pause_timeline=True)
    img = Image.fromarray(np.asarray(rgb.get_data())[:, :, :3].astype(np.uint8)).convert("RGBA")
    if args.clean:
        side.append({"orbit_min": round(phase * ORBIT_S / 60.0, 2)})
        img.convert("RGB").save(fdir / f"f{k:05d}.png")
        continue
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    mins = phase * ORBIT_S / 60.0
    lines = [(f"View from the ISS Cupola position · one orbit ≈ 92.5 min in {args.seconds:g} s · orbit time {mins:5.1f} min",
              font),
             ("visual only · Earth: NASA Blue Marble NG, 1:1000 scale at 400 km · Sun turns at the orbital rate · "
              "Cupola mesh hidden · no orbital mechanics", small)]
    y = 18
    dr.rectangle([0, 0, img.size[0], 18 + sum(int(f.size * 1.4) for _, f in lines) + 6], fill=(0, 0, 0, 110))
    for text, f in lines:
        dr.text((20, y), text, font=f, fill=(255, 255, 255, 235))
        y += int(f.size * 1.4)
    Image.alpha_composite(img, ov).convert("RGB").save(fdir / f"f{k:05d}.png")
mp4 = args.out or OUT / "cupola_earth.mp4"
if args.clean:
    mp4.with_name(mp4.stem + "_frames.json").write_text(json.dumps(
        {"orbit_s": ORBIT_S, "fps": args.fps, "speed": span * ORBIT_S / args.seconds, "frames": side}),
        encoding="utf-8")
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps), "-i", str(fdir / "f%05d.png"),
                "-c:v", "libx264", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)], check=True)
print(json.dumps({"video": str(mp4), "frames": n, "fps": args.fps}))
app.close()

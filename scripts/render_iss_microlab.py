"""Render S1 ISS Micro-Lab v0 stills and the LinkedIn clip with Isaac Sim RTX (path tracing).

C:\\isaacsim\\python.bat scripts/render_iss_microlab.py --mode stills|video|quick [--subframes N]
Outputs go to outputs/iss_microlab_v0/ (gitignored). Overlays are typography only; no telemetry numbers.
"""
import argparse
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_microlab_v0.usda"
OUT = ROOT / "outputs" / "iss_microlab_v0"
W, H, FPS = 1920, 1080, 30

ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["stills", "video", "quick"], default="stills")
ap.add_argument("--subframes", type=int, default=48)
args = ap.parse_args()
if args.mode == "quick":
    W, H = 960, 540

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_iss_microlab import CAMERA, look_at_xform  # noqa: E402

RENDER = {"rendermode": "PathTracing", "spp_per_subframe": 1, "subframes": args.subframes,
          "denoiser": True, "resolution": [W, H], "fstop": 0.0, "motion_blur": False}
s = carb.settings.get_settings()
s.set("/rtx/rendermode", "PathTracing")
s.set("/rtx/pathtracing/spp", 1)
s.set("/rtx/pathtracing/totalSpp", args.subframes)
s.set("/rtx/pathtracing/optixDenoiser/enabled", True)
s.set("/rtx/post/motionblur/enabled", False)
s.set("/rtx/post/dof/enabled", False)
s.set("/rtx/post/lensFlares/enabled", False)
s.set("/rtx/post/histogram/enabled", False)   # fixed exposure: layers must be comparable

ctx = omni.usd.get_context()
if not ctx.open_stage(str(SCENE)):
    app.close()
    raise SystemExit(f"failed to open {SCENE}")
for _ in range(60):
    app.update()
stage = ctx.get_stage()
print("stage opened", stage.GetRootLayer().identifier, flush=True)
world = stage.GetPrimAtPath("/World")
cam = stage.GetPrimAtPath("/World/Cameras/Hero")
cam_op = UsdGeom.Xformable(cam).GetOrderedXformOps()[0]
rp = rep.create.render_product(str(cam.GetPath()), (W, H))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach(rp)
rep.orchestrator.set_capture_on_play(False)


def set_layer(name):
    world.GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection(name)


def set_camera(eye, target):
    cam_op.Set(look_at_xform(eye, target))


def render(layer, eye=CAMERA["eye"], target=CAMERA["target"]):
    set_layer(layer)
    set_camera(eye, target)
    app.update()
    rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0, pause_timeline=True)
    data = rgb.get_data()
    return Image.fromarray(np.asarray(data)[:, :, :3].astype(np.uint8), "RGB")


# ---- typography overlay (restrained: title, state tag, layer indicator) -----------------------
FONT_DIR = Path("C:/Windows/Fonts")
k = W / 1920


def font(name, size):
    for n in (name, "segoeui.ttf"):
        p = FONT_DIR / n
        if p.exists():
            return ImageFont.truetype(str(p), int(size * k))
    return ImageFont.load_default()


F_TITLE, F_SUB = font("seguisb.ttf", 44), font("segoeui.ttf", 21)  # Segoe has the arrow glyph; Bahnschrift does not
F_TAG, F_LAYER, F_NOTE = font("bahnschrift.ttf", 19), font("bahnschrift.ttf", 24), font("segoeui.ttf", 19)
NOTES = {
    "visual": "Render geometry  ·  ISS-inspired authored bay, not a digital twin",
    "collision": "Engineering collision proxies  ·  R2 hidden: no validated collision / physics model",
    "affordance": "Task affordances  ·  graspable handrails  ·  maintenance target  ·  approach waypoint",
}


def overlay(img, layer=None, title=1.0, layer_alpha=1.0):
    base = img.convert("RGBA")
    ov = Image.new("RGBA", base.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    m = int(56 * k)

    gray = base.convert("L")

    def ink(box):
        """Dark ink on bright interior walls, light ink on dark proxies: keeps contrast without a backing plate."""
        x0, y0, x1, y1 = [int(v) for v in box]
        crop = gray.crop((max(0, x0), max(0, y0), min(W, x1), min(H, y1)))
        mean = sum(i * c for i, c in enumerate(crop.histogram())) / max(1, crop.width * crop.height)
        return ((24, 28, 34), (255, 255, 255)) if mean > 150 else ((255, 255, 255), (0, 0, 0))

    def text(xy, s, f, a, fill=None):
        w_, h_ = d.textlength(s, font=f), f.size * 1.3
        fg, sh = ink((xy[0], xy[1], xy[0] + w_, xy[1] + h_)) if fill is None else (fill, (0, 0, 0))
        d.text((xy[0] + 1, xy[1] + 1), s, font=f, fill=sh + (int(90 * a),))
        d.text(xy, s, font=f, fill=fg + (int(235 * a),))

    if title > 0:
        text((m, m), "ISS → LUNAR PHYSICAL AI", F_TITLE, title)
        text((m, m + int(58 * k)), "S1 · ISS Micro-Lab v0 · OpenUSD / Isaac Sim", F_SUB, title * 0.85)
    tag = "KINEMATIC / VISUAL"
    tw = d.textlength(tag, font=F_TAG)
    x0, y0 = W - m - tw - int(24 * k), m
    tag_box = [x0, y0, x0 + tw + int(24 * k), y0 + int(36 * k)]
    fg, _ = ink(tag_box)
    d.rectangle(tag_box, outline=fg + (190,), width=max(1, int(1.5 * k)))
    text((x0 + int(12 * k), y0 + int(6 * k)), tag, F_TAG, 1.0, fill=fg)
    if layer and layer_alpha > 0:
        x, y = m, H - m - int(30 * k)
        for name in ("visual", "collision", "affordance"):
            a = layer_alpha if name == layer else layer_alpha * 0.38
            text((x, y), name.upper(), F_LAYER, a)
            x += d.textlength(name.upper(), font=F_LAYER) + int(30 * k)
        text((m, y - int(34 * k)), NOTES[layer], F_NOTE, layer_alpha * 0.9)
    return Image.alpha_composite(base, ov).convert("RGB")


def eye_at(t):
    """Slow, legible push-in: 0.12 m forward, 0.04 m toward the aisle centre; constant target keeps R2 framed."""
    e = CAMERA["eye"]
    u = 0.5 - 0.5 * math.cos(math.pi * t)  # ease in/out
    return (e[0] + 0.12 * u, e[1] - 0.04 * u, e[2])


def ramp(f, a, b):
    return min(1.0, max(0.0, (f - a) / float(b - a)))


SCHEDULE = [(0, 135, "visual"), (135, 195, "collision"), (195, 255, "affordance"), (255, 300, "visual")]
XFADE = 12


def frame_plan(f):
    """Return [(layer, weight)], title alpha, layer-label alpha, label layer for clip frame f (0..299)."""
    layers = None
    for b in (135, 195, 255):
        if b - XFADE // 2 <= f < b + XFADE // 2:
            w = (f - (b - XFADE // 2) + 0.5) / XFADE
            before = [n for s0, s1, n in SCHEDULE if s0 <= b - 1 < s1][0]
            after = [n for s0, s1, n in SCHEDULE if s0 <= b < s1][0]
            layers = [(before, 1 - w), (after, w)]
    if layers is None:
        layers = [([n for s0, s1, n in SCHEDULE if s0 <= f < s1][0], 1.0)]
    title = 1.0 - ramp(f, 60, 75) + ramp(f, 258, 273)
    label = ramp(f, 75, 90) * (1.0 - ramp(f, 252, 264))
    dom = max(layers, key=lambda lw: lw[1])[0]
    return layers, min(title, 1.0), label, dom


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    receipt = {"scene": str(SCENE.relative_to(ROOT)), "render": RENDER, "camera": CAMERA, "fps": FPS, "files": {}}
    if args.mode == "quick":
        for layer in ("visual", "collision", "affordance"):
            render(layer).save(OUT / f"quick_{layer}.png")
    elif args.mode == "stills":
        vis = render("visual")
        vis.save(OUT / "visual_clean.png")
        overlay(vis, None, title=1.0).save(OUT / "hero.png")
        overlay(vis, "visual", title=0.0).save(OUT / "layer_visual.png")
        for layer in ("collision", "affordance"):
            overlay(render(layer), layer, title=0.0).save(OUT / f"layer_{layer}.png")
        receipt["files"] = {n: f"outputs/iss_microlab_v0/{n}" for n in
                            ("hero.png", "visual_clean.png", "layer_visual.png", "layer_collision.png", "layer_affordance.png")}
    else:
        frames = OUT / "frames"
        shutil.rmtree(frames, ignore_errors=True)
        frames.mkdir(parents=True)
        n = 10 * FPS
        for f in range(n):
            eye = eye_at(f / (n - 1))
            layers, title, label, dom = frame_plan(f)
            acc = None
            for layer, w in layers:
                img = np.asarray(render(layer, eye), dtype=np.float32)
                acc = img * w if acc is None else acc + img * w
            img = Image.fromarray(np.clip(acc, 0, 255).astype(np.uint8))
            overlay(img, dom, title=title, layer_alpha=label).save(frames / f"f{f:04d}.png")
            if f % 30 == 0:
                print(f"frame {f}/{n} {time.time() - t0:.0f}s", flush=True)
        mp4 = OUT / "iss_microlab_v0_linkedin_1080p30.mp4"
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(frames / "f%04d.png"),
               "-c:v", "libx264", "-preset", "slow", "-crf", "17", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)]
        subprocess.run(cmd, check=True)
        receipt["files"] = {"video": f"outputs/iss_microlab_v0/{mp4.name}", "frames": n, "ffmpeg": " ".join(cmd[:-1])}
    receipt["seconds"] = round(time.time() - t0, 1)
    (OUT / f"render_receipt_{args.mode}.json").write_text(json.dumps(receipt, indent=1), encoding="utf-8")
    print(json.dumps(receipt, indent=1))


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"render_error_{args.mode}.txt").write_text(traceback.format_exc(), encoding="utf-8")
    raise
finally:
    app.close()

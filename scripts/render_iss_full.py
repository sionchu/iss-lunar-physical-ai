"""Render the full ISS interior scene v0 and measure its cost in Isaac Sim (headless).

C:\\isaacsim\\python.bat scripts/render_iss_full.py --mode stills [--subframes 64] [--layer visual]
C:\\isaacsim\\python.bat scripts/render_iss_full.py --mode perf
stills: one RTX path-traced 1920x1080 still per camera in /World/Cameras (one wide view per module + the
        through-hatch view), no overlay, plus a labelled contact sheet.
perf:   GPU memory (nvidia-smi, whole GPU: before app, empty app, scene open) and render FPS with RTX real-time at
        1920x1080 from two cameras, timeline stopped and playing (physics steps with static colliders only).
Outputs: outputs/iss_full_v0/ (gitignored).
"""
import argparse
import json
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_full_v0.usda"
OUT = ROOT / "outputs" / "iss_full_v0"
W, H = 1920, 1080
ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["stills", "perf"], default="stills")
ap.add_argument("--subframes", type=int, default=64)
ap.add_argument("--layer", default="visual")
ap.add_argument("--camera", action="append", help="render only these camera names (repeatable)")
args = ap.parse_args()


def gpu_mem():
    q = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.used,memory.total,driver_version",
                        "--format=csv,noheader,nounits"], capture_output=True, text=True).stdout.strip().split(", ")
    return {"gpu": q[0], "used_mib": int(q[1]), "total_mib": int(q[2]), "driver": q[3]}


mem = {"before_app": gpu_mem()}
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": W, "height": H})
import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

for _ in range(20):
    app.update()
mem["empty_app"] = gpu_mem()
s = carb.settings.get_settings()
if args.mode == "stills":
    s.set("/rtx/rendermode", "PathTracing")
    s.set("/rtx/pathtracing/spp", 1)
    s.set("/rtx/pathtracing/totalSpp", args.subframes)
    s.set("/rtx/pathtracing/optixDenoiser/enabled", True)
else:
    s.set("/rtx/rendermode", "RaytracedLighting")
for k in ("/rtx/post/motionblur/enabled", "/rtx/post/dof/enabled", "/rtx/post/lensFlares/enabled", "/rtx/post/histogram/enabled"):
    s.set(k, False)

t0 = time.time()
ctx = omni.usd.get_context()
if not ctx.open_stage(str(SCENE)):
    app.close()
    raise SystemExit(f"failed to open {SCENE}")
stage = ctx.get_stage()
for _ in range(60):
    app.update()
open_s = time.time() - t0
stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection(args.layer)
cams = [p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Cameras")) if p.IsA(UsdGeom.Camera)
        and (not args.camera or p.GetName() in args.camera)]
rp = rep.create.render_product(str(cams[0].GetPath()), (W, H))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach(rp)
rep.orchestrator.set_capture_on_play(False)


def use_camera(path):
    """Retarget the render product (UsdRender product prim, session layer) to another camera."""
    from pxr import Sdf
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        stage.GetPrimAtPath(rp.path).GetRelationship("camera").SetTargets([Sdf.Path(path)])
    got = stage.GetPrimAtPath(rp.path).GetRelationship("camera").GetTargets()
    if [str(x) for x in got] != [path]:
        raise RuntimeError(f"render product camera not switched: {got}")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    receipt = {"scene": str(SCENE.relative_to(ROOT)), "mode": args.mode, "layer": args.layer, "resolution": [W, H],
               "rendermode": s.get("/rtx/rendermode"), "stage_open_s": round(open_s, 1), "files": {}}
    if args.mode == "stills":
        receipt["render"] = {"spp_per_subframe": 1, "subframes": args.subframes, "denoiser": True, "fstop": 0.0}
        shots = []
        for cam in cams:
            use_camera(str(cam.GetPath()))
            app.update()
            rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0, pause_timeline=True)
            img = Image.fromarray(np.asarray(rgb.get_data())[:, :, :3].astype(np.uint8), "RGB")
            name = f"{cam.GetName()}{'' if args.layer == 'visual' else '_' + args.layer}.png"
            img.save(OUT / name)
            receipt["files"][cam.GetName()] = f"outputs/iss_full_v0/{name}"
            shots.append((cam.GetName(), img))
            print("rendered", name, flush=True)
        cols = 3
        tw, th = W // 3, H // 3
        sheet = Image.new("RGB", (cols * tw, -(-len(shots) // cols) * th), (12, 12, 14))
        d = ImageDraw.Draw(sheet)
        try:
            f = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 22)
        except OSError:
            f = ImageFont.load_default()
        for i, (n, img) in enumerate(shots):
            x, y = (i % cols) * tw, (i // cols) * th
            sheet.paste(img.resize((tw, th)), (x, y))
            d.text((x + 12, y + 8), n, font=f, fill=(255, 255, 255))
        sheet.save(OUT / f"contact_sheet_{args.layer}.png")
        receipt["files"]["contact_sheet"] = f"outputs/iss_full_v0/contact_sheet_{args.layer}.png"
    else:
        mem["scene_open"] = gpu_mem()
        timeline = omni.timeline.get_timeline_interface()
        fps = {}
        for cam in (cams[0], cams[-1]):
            use_camera(str(cam.GetPath()))
            for state in ("stopped", "playing"):
                if state == "playing":
                    t1 = time.time()
                    timeline.play()
                    app.update()
                    fps[f"first_play_update_s_{cam.GetName()}"] = round(time.time() - t1, 2)
                for _ in range(60):
                    app.update()
                n, t1 = 300, time.time()
                for _ in range(n):
                    app.update()
                fps[f"{cam.GetName()}_{state}"] = round(n / (time.time() - t1), 1)
                if state == "playing":
                    timeline.stop()
                    app.update()
            mem[f"after_{cam.GetName()}"] = gpu_mem()
        receipt["fps_app_update"] = fps
        receipt["fps_note"] = ("headless app.update() rate with one 1920x1080 render product attached; GUI viewport "
                               "rate NOT measured")
    receipt["gpu_memory"] = mem
    receipt["gpu_memory_note"] = "nvidia-smi whole-GPU memory.used (other processes included); deltas are indicative"
    path = OUT / f"render_receipt_{args.mode}{'' if args.layer == 'visual' else '_' + args.layer}.json"
    path.write_text(json.dumps(receipt, indent=1), encoding="utf-8")
    print(json.dumps(receipt, indent=1), flush=True)


try:
    main()
finally:
    app.close()

"""Convert the NASA Astrobee media meshes (7 ISS interior modules + 4 ISS handrail lengths) to USD.

Source: github.com/nasa/astrobee_media @ 1cb0620. Licensing is recorded as found, two statements:
astrobee_iss/package.xml declares Apache-2.0, while the repository readme states that all multimedia is covered by
the NASA Media Usage Guidelines (interior meshes redistributed from NASA 3D Resources "ISS Internal" by IGOAL,
handrails from NASA JSC r2_gazebo). Raw files stay in gitignored assets/raw/; converted USD goes to gitignored
assets/cache/iss_media/. Existing conversions are kept unless --force (the S1 micro-lab references two of them).
Run: C:\\isaacsim\\python.bat scripts/import_iss_media.py [--force]
"""
import argparse
import asyncio
import hashlib
import json
import subprocess
from pathlib import Path

from isaacsim import SimulationApp

ap = argparse.ArgumentParser()
ap.add_argument("--force", action="store_true", help="re-convert items whose USD already exists")
args = ap.parse_args()
app = SimulationApp({"headless": True})

import omni.kit.asset_converter as ac  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / "assets" / "raw" / "astrobee_media"
OUT = ROOT / "assets" / "cache" / "iss_media"
MODULES = ("us_lab", "node_1", "node_2", "node_3", "eu_lab", "jpm", "cupola")
HANDRAILS = ("handrail_8_5", "handrail_21_5", "handrail_30", "handrail_41_5")
ITEMS = {**{m: RAW / "astrobee_iss" / "meshes" / f"{m}.dae" for m in MODULES},
         **{h: RAW / f"astrobee_{h}" / "meshes" / f"{h}.dae" for h in HANDRAILS}}


async def convert(src, dst):
    ctx = ac.AssetConverterContext()
    ctx.ignore_animations = True
    ctx.ignore_camera = True
    ctx.ignore_light = True
    ctx.merge_all_meshes = False
    ctx.use_meter_as_world_unit = True
    task = ac.get_instance().create_converter_task(str(src), str(dst), None, ctx)
    ok = await task.wait_until_finished()
    return ok, task.get_error_message()


def describe(dst):
    st = Usd.Stage.Open(str(dst))
    rng = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
        st.GetPseudoRoot()).ComputeAlignedRange()
    meshes = [UsdGeom.Mesh(p) for p in st.Traverse() if p.IsA(UsdGeom.Mesh)]
    tris = sum(sum(c - 2 for c in (m.GetFaceVertexCountsAttr().Get() or [])) for m in meshes)
    return {"upAxis": UsdGeom.GetStageUpAxis(st), "metersPerUnit": UsdGeom.GetStageMetersPerUnit(st),
            "bbox_min": [round(v, 4) for v in rng.GetMin()], "bbox_max": [round(v, 4) for v in rng.GetMax()],
            "meshes": len(meshes), "triangles": tris,
            "usd_sha256": hashlib.sha256(dst.read_bytes()).hexdigest()}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    commit = subprocess.run(["git", "-C", str(RAW), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    report = {"source_repo": "https://github.com/nasa/astrobee_media", "source_commit": commit or "UNKNOWN",
              "license": {"package_xml": "Apache-2.0 (astrobee_iss/package.xml and each handrail package.xml)",
                          "readme": "All multimedia is covered by NASA's Media Usage Guidelines "
                                    "(https://www.nasa.gov/multimedia/guidelines/index.html)",
                          "status": "both statements recorded; which governs the meshes is UNKNOWN"},
              "upstream": {"interior": "NASA 3D Resources ISS Internal (IGOAL), https://nasa3d.arc.nasa.gov/detail/iss-internal",
                           "handrails": "nasa-jsc-robotics/r2_gazebo"},
              "converter": "omni.kit.asset_converter (metres, animations/cameras/lights ignored, meshes not merged)",
              "items": {}}
    for name, src in ITEMS.items():
        dst = OUT / f"{name}.usd"
        info = {"src": src.relative_to(ROOT).as_posix(), "src_sha256": hashlib.sha256(src.read_bytes()).hexdigest(),
                "usd": dst.relative_to(ROOT).as_posix()}
        if dst.exists() and not args.force:
            info.update({"ok": True, "error": "", "converted": "kept existing"})
        else:
            ok, err = asyncio.get_event_loop().run_until_complete(convert(src, dst))
            info.update({"ok": bool(ok), "error": err, "converted": "now"})
        if info["ok"]:
            info.update(describe(dst))
        report["items"][name] = info
        print(json.dumps({name: info}), flush=True)
    (OUT / "import_report.json").write_text(json.dumps(report, indent=1), encoding="utf-8")


try:
    main()
finally:
    app.close()

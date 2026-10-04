"""Convert the NASA Astrobee media meshes used by S1 (US Lab interior + one ISS handrail) to USD.

Source: github.com/nasa/astrobee_media (Apache-2.0; interior meshes from NASA 3D Resources "ISS Internal" by IGOAL,
handrails from NASA JSC r2_gazebo). Raw files stay in gitignored assets/raw/; converted USD goes to gitignored
assets/cache/iss_media/. Run: C:\\isaacsim\\python.bat scripts/import_iss_media.py
"""
import asyncio
import hashlib
import json
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.asset_converter as ac  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / "assets" / "raw" / "astrobee_media"
OUT = ROOT / "assets" / "cache" / "iss_media"
ITEMS = {
    "us_lab": RAW / "astrobee_iss" / "meshes" / "us_lab.dae",
    "handrail_30": next((RAW / "astrobee_handrail_30").rglob("*.dae")),
}


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


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    report = {"source_repo": "https://github.com/nasa/astrobee_media", "license": "Apache-2.0 (astrobee_iss/package.xml)",
              "upstream": {"interior": "NASA 3D Resources ISS Internal (IGOAL)", "handrails": "nasa-jsc-robotics/r2_gazebo"},
              "items": {}}
    for name, src in ITEMS.items():
        dst = OUT / f"{name}.usd"
        ok, err = asyncio.get_event_loop().run_until_complete(convert(src, dst))
        info = {"src": str(src.relative_to(ROOT)), "src_sha256": hashlib.sha256(src.read_bytes()).hexdigest(),
                "usd": str(dst.relative_to(ROOT)), "ok": bool(ok), "error": err}
        if ok:
            st = Usd.Stage.Open(str(dst))
            rng = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
                st.GetPseudoRoot()).ComputeAlignedRange()
            info.update({"upAxis": UsdGeom.GetStageUpAxis(st), "metersPerUnit": UsdGeom.GetStageMetersPerUnit(st),
                         "bbox_min": [round(v, 3) for v in rng.GetMin()], "bbox_max": [round(v, 3) for v in rng.GetMax()],
                         "meshes": sum(1 for p in st.Traverse() if p.IsA(UsdGeom.Mesh))})
        report["items"][name] = info
        print(json.dumps({name: info}), flush=True)
    (OUT / "import_report.json").write_text(json.dumps(report, indent=1), encoding="utf-8")


try:
    main()
finally:
    app.close()

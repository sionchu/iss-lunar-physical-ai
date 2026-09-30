"""Headless Isaac Sim 6.0.1 import probe for the unmodified full R2 URDF."""

import json
from pathlib import Path
import sys
import traceback

from isaacsim import SimulationApp


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets/raw/nasa-urdf-robots/r2_description/robots/r2c6.urdf"
OUT = ROOT / "outputs/r2_isaac_import"
OUT.mkdir(parents=True, exist_ok=True)
result = {"source_urdf": str(URDF), "fix_base": False, "gravity_magnitude": 0.0,
          "import_success": False, "articulation_found": False, "dof_count": None,
          "physx_total_mass_kg": None, "warnings": [], "error": None}
app = None
try:
    app = SimulationApp({"headless": True})
    import omni.kit.app
    import omni.timeline
    import omni.usd
    from pxr import UsdPhysics
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
    from isaacsim.core.experimental.prims import Articulation

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("omni.scene.optimizer.core", True)
    manager.set_extension_enabled_immediate("isaacsim.robot.schema", True)
    config = URDFImporterConfig()
    config.urdf_path = str(URDF)
    config.usd_path = str(OUT)
    config.ros_package_paths = [{"name": "r2_description", "path": str(URDF.parents[1])}]
    config.fix_base = False
    usd_path = URDFImporter(config).import_urdf()
    if not usd_path:
        raise RuntimeError("URDFImporter.import_urdf returned no USD path")
    result["import_success"] = True
    result["usd_path"] = str(usd_path)
    context = omni.usd.get_context()
    if not context.open_stage(str(usd_path)):
        raise RuntimeError("USD stage could not be opened")
    for _ in range(5):
        app.update()
    stage = context.get_stage()
    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr((0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(0.0)
    roots = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    result["articulation_roots"] = roots
    result["articulation_found"] = bool(roots)
    if not roots:
        raise RuntimeError("Imported stage has no ArticulationRootAPI")
    articulation = Articulation(roots[0])
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(12):
        app.update()
    result["dof_count"] = articulation.num_dofs
    if not articulation.is_physics_tensor_entity_valid():
        raise RuntimeError("PhysX articulation tensor entity is invalid")
    masses = articulation.get_link_masses().numpy()[0]
    result["physx_total_mass_kg"] = float(masses.sum())
    result["physx_link_count"] = int(len(masses))
    timeline.stop()
except Exception as exc:
    result["error"] = f"{type(exc).__name__}: {exc}"
    traceback.print_exc()
finally:
    (ROOT / "outputs/r2_isaac_import_result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print("R2_IMPORT_RESULT=" + json.dumps(result), flush=True)
    if app is not None:
        app.close()
sys.exit(0 if result["import_success"] and result["articulation_found"] and result["dof_count"] is not None and result["physx_total_mass_kg"] is not None else 1)

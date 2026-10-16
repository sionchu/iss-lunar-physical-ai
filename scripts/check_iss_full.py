"""Scene-contract checks for the full ISS interior scene v0 (scenes/iss_full_v0.usda).

C:\\isaacsim\\python.bat scripts/check_iss_full.py            # static pxr checks
C:\\isaacsim\\python.bat scripts/check_iss_full.py --isaac    # + open in Isaac Sim (Kit) and re-check
Same contract as scripts/check_iss_microlab.py (paths, units, zero-g, truth-layer variants, collision independence,
affordance relationships), plus module poses vs the Astrobee URDF, collision strategy, and hatch alignment re-measured
on the composed stage. Writes outputs/iss_full_v0/scene_check[_isaac].json; exit code 0 only if every check passes.
"""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_full_v0.usda"
ap = argparse.ArgumentParser()
ap.add_argument("--isaac", action="store_true")
args = ap.parse_args()

if args.isaac:
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import omni.usd
    ctx = omni.usd.get_context()
    ok_open = ctx.open_stage(str(SCENE))
    for _ in range(30):
        app.update()
    stage = ctx.get_stage()
else:
    from pxr import Usd
    stage = Usd.Stage.Open(str(SCENE))
    ok_open = stage is not None

import numpy as np  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_iss_full as B  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append({"check": name, "pass": bool(ok), "detail": detail})


ISS = B.ISS
check("scene opens", ok_open and stage is not None, str(SCENE.relative_to(ROOT)))
REQUIRED = ["/World", f"{ISS}/Visual", f"{ISS}/Collision", f"{ISS}/Affordances", f"{ISS}/Lighting", "/World/Actors",
            "/World/Task/Targets/MaintenancePanel", "/World/Cameras", "/World/PhysicsScene"]
missing = [p for p in REQUIRED if not stage.GetPrimAtPath(p).IsValid()]
check("required prim paths", not missing, f"missing={missing}")
check("units: metersPerUnit=1, upAxis=Z", UsdGeom.GetStageMetersPerUnit(stage) == 1.0 and UsdGeom.GetStageUpAxis(stage) == "Z")
g = UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene")).GetGravityMagnitudeAttr().Get()
check("microgravity physics scene", g == 0.0, f"gravity={g}")
vset = stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("truthLayer")
check("truth-layer variant set", sorted(vset.GetVariantNames()) == sorted(B.LAYERS), f"{vset.GetVariantNames()}")
vset.SetVariantSelection("visual")

# station frame and module poses
iss_ops = [(o.GetOpName(), o.Get()) for o in UsdGeom.Xformable(stage.GetPrimAtPath(ISS)).GetOrderedXformOps()]
check("ISS root = Astrobee world frame rotated 180 deg about x", iss_ops == [("xformOp:rotateX:stationToScene", 180.0)],
      str(iss_ops))
poses = B.urdf_poses()
bad, refs = [], {}
for name, (mesh, _) in B.MODULES.items():
    prim = stage.GetPrimAtPath(f"{ISS}/Visual/{name}")
    tmp_stage = Usd.Stage.CreateInMemory()
    tmp = tmp_stage.DefinePrim("/m", "Xform")
    B.pose_ops(tmp, poses[mesh]["visual"])
    want = UsdGeom.Xformable(tmp).GetLocalTransformation()
    got = UsdGeom.Xformable(prim).GetLocalTransformation()
    err = max(abs(want[i][j] - got[i][j]) for i in range(4) for j in range(4))
    items = prim.GetPrimStack()[0].referenceList.prependedItems if prim.GetPrimStack() else []
    refs[name] = [str(r.assetPath) for r in items]
    if err > 1e-9 or not any(r.endswith(f"{mesh}.usd") for r in refs[name]):
        bad.append(f"{name}: matrix err {err:.2e} refs {refs[name]}")
check("7 module meshes referenced at the URDF <visual> poses", not bad and len(refs) == 7, f"bad={bad}")
lab = stage.GetPrimAtPath(f"{ISS}/Visual/USLab")
check("US Lab collision-vs-visual origin recorded and flagged UNKNOWN",
      lab.HasAttribute("iss:collisionOriginXyz") and "UNKNOWN" in (lab.GetAttribute("iss:originStatus").Get() or ""),
      f"visual {poses['us_lab']['visual']['xyz']} collision {poses['us_lab']['collision']['xyz']}")
gaps = stage.GetPrimAtPath(ISS).GetAttribute("iss:knownGaps").Get()
check("missing ISS elements recorded as gaps", gaps and len(gaps) >= 4, str(list(gaps or [])))
check("license statements recorded (Apache-2.0 + NASA Media Usage Guidelines)",
      all(s in (stage.GetPrimAtPath(ISS).GetAttribute("iss:license").Get() or "") for s in ("Apache-2.0", "NASA Media Usage")))

# collision layer
col_root, vis_root = f"{ISS}/Collision", f"{ISS}/Visual"
col = [p for p in Usd.PrimRange(stage.GetPrimAtPath(col_root)) if p.IsA(UsdGeom.Gprim)]
vis = [p for p in Usd.PrimRange(stage.GetPrimAtPath(vis_root), Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Gprim)]
check("collision gprims all have CollisionAPI", col and all(p.HasAPI(UsdPhysics.CollisionAPI) for p in col), f"n={len(col)}")
check("visual gprims carry no CollisionAPI", vis and not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in vis), f"n={len(vis)}")
meshes = [p for p in col if p.IsA(UsdGeom.Mesh)]
approx = sorted({UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() for p in meshes})
sdf = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPseudoRoot())
       if any("SDF" in s or "Sdf" in s for s in p.GetAppliedSchemas())]
check("mesh colliders are static triangle meshes (approximation none), no SDF colliders",
      meshes and approx == ["none"] and not sdf, f"meshes={len(meshes)} approximation={approx} sdf={sdf[:3]}")
rigid = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(ISS)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
check("environment colliders are static (no rigid bodies)", not rigid, str(rigid[:3]))
strat = {n: stage.GetPrimAtPath(f"{col_root}/{n}").GetAttribute("iss:collisionStrategy").Get() for n in B.MODULES}
check("collision strategy recorded for every module", all(strat.values()), json.dumps({k: (v or "")[:40] for k, v in strat.items()}))

# handrails (measured in the collision layer: BBoxCache skips invisible prims)
vset.SetVariantSelection("collision")
bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], useExtentsHint=False)
rails = {}
for name in ("Handrail_A", "Handrail_B"):
    def ext(path):
        r = bbc.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
        return sorted([round(v, 4) for v in (r.GetMax() - r.GetMin())], reverse=True)
    grip, full = ext(f"{col_root}/USLab/{name}/Grip"), ext(f"{col_root}/USLab/{name}")
    rails[name] = {"grip_m": grip, "assembly_m": full}
    check(f"{name} dimensions plausible (as S1)", 0.30 <= grip[0] <= 0.90 and 0.015 <= grip[1] <= 0.05
          and 0.06 <= full[1] <= 0.12, json.dumps(rails[name]))
vset.SetVariantSelection("visual")
bbc.Clear()
lab_box = bbc.ComputeWorldBound(lab).ComputeAlignedRange()
inside = [n for n in ("Handrail_A", "Handrail_B")
          if lab_box.Contains(bbc.ComputeWorldBound(stage.GetPrimAtPath(f"{vis_root}/USLab_S1Frame/{n}")).ComputeAlignedRange())]
check("S1 handrails carried inside the US Lab mesh bounds", len(inside) == 2, str(inside))
others = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(vis_root))
          if "handrail" in str(p.GetPath()).lower() and "USLab_S1Frame" not in str(p.GetPath())]
check("no handrails invented outside the US Lab", not others, str(others[:3]))

# affordances
aff = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{ISS}/Affordances")) if p.HasAttribute("iss:affordance")]
broken = [f"{p.GetPath()}:{rel}" for p in aff for rel in ("iss:collisionProxy", "iss:visualSource")
          if len(p.GetRelationship(rel).GetTargets()) != 1 or not stage.GetPrimAtPath(p.GetRelationship(rel).GetTargets()[0]).IsValid()]
kinds = sorted(p.GetAttribute("iss:affordance").Get() for p in aff)
check("affordances resolve to real collision + visual prims", aff and not broken, f"kinds={kinds} broken={broken}")
check("2 graspable rails + maintenance target + hatch affordances",
      kinds.count("graspable") == 2 and "maintenance_target" in kinds and kinds.count("hatch") >= 1, str(kinds))

# hatch alignment, re-measured on the composed stage
rows = B.measure_hatches(stage, vis_root)
for r in rows:
    if r["within_tolerance"] is None:
        check(f"hatch {r['pair']} measurable", False, f"a={(r['a'] or {}).get('status')} b={(r['b'] or {}).get('status')}")
    else:
        check(f"hatch {r['pair']} aligned (centre offset <= {B.HATCH_TOL['centre_offset_m']} m, "
              f"|axial gap| <= {B.HATCH_TOL['axial_gap_m']} m)", r["within_tolerance"],
              f"offset {r['centre_offset_m']} m, gap {r['axial_gap_m']} m, openings {r['a']['size_m']} / {r['b']['size_m']}")

# cameras and lighting
cams = [p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Cameras")) if p.IsA(UsdGeom.Camera)]
outside = []
for p in cams:
    eye = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0).ExtractTranslation()
    mod = p.GetName().split("_")[0] if not p.GetName().startswith("JEM_PM") else "JEM_PM"
    box = bbc.ComputeWorldBound(stage.GetPrimAtPath(f"{vis_root}/{mod}")).ComputeAlignedRange()
    if not box.Contains(eye):
        outside.append(p.GetName())
check("one wide camera per module + through-hatch view, eyes inside their module bounds",
      len(cams) == len(B.MODULES) + 1 and not outside, f"cameras={[p.GetName() for p in cams]} outside={outside}")
lights = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{ISS}/Lighting")) if p.HasAPI(UsdLux.LightAPI)]
check("lighting resolves", len(lights) >= len(B.MODULES), f"lights={len(lights)}")

for name, flags in B.LAYERS.items():
    vset.SetVariantSelection(name)
    state = [UsdGeom.Imageable(stage.GetPrimAtPath(p)).ComputeVisibility() != "invisible" for p in B.LAYER_PRIMS]
    check(f"layer '{name}' visibility", state == list(flags), str(dict(zip(B.LAYER_PRIMS, state))))
vset.SetVariantSelection("visual")

gate = subprocess.run(["git", "diff", "--quiet", "HEAD", "--", "configs/iss_v0_momentum.json",
                       "configs/iss_v0_momentum_v2_armature.json", "configs/iss_v0_momentum_v3.json"], cwd=ROOT).returncode
check("momentum gate configs unchanged vs HEAD", gate == 0)

out = ROOT / "outputs" / "iss_full_v0"
out.mkdir(parents=True, exist_ok=True)
report = {"scene": str(SCENE.relative_to(ROOT)), "mode": "isaac" if args.isaac else "pxr", "handrails": rails,
          "hatches": rows, "passed": sum(r["pass"] for r in results), "total": len(results), "results": results}
(out / f"scene_check{'_isaac' if args.isaac else ''}.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
for r in results:
    print(("PASS " if r["pass"] else "FAIL ") + r["check"] + (f"  [{r['detail']}]" if r["detail"] else ""))
print(f"{report['passed']}/{report['total']} checks passed")
code = 0 if report["passed"] == report["total"] else 1
if args.isaac:
    app.close()
sys.exit(code)

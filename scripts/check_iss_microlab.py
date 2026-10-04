"""Scene-contract checks for S1 ISS Micro-Lab v0 only (not a generic framework).

C:\\isaacsim\\python.bat scripts/check_iss_microlab.py            # static pxr checks
C:\\isaacsim\\python.bat scripts/check_iss_microlab.py --isaac    # + open in Isaac Sim (Kit) and re-check
Exit code 0 only if every check passes. Writes outputs/iss_microlab_v0/scene_check[_isaac].json.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_microlab_v0.usda"
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

from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_iss_microlab import CHAMFER, HALF, LAYERS, LAYER_PRIMS  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append({"check": name, "pass": bool(ok), "detail": detail})


check("scene opens", ok_open and stage is not None, str(SCENE.relative_to(ROOT)))
REQUIRED = ["/World", "/World/Environment/ISS/Visual", "/World/Environment/ISS/Collision",
            "/World/Environment/ISS/Affordances", "/World/Environment/ISS/Lighting", "/World/Actors/R2_Visual",
            "/World/Task/Targets/MaintenancePanel", "/World/Task/Waypoints/R2_Park", "/World/Cameras/Hero",
            "/World/PhysicsScene"]
missing = [p for p in REQUIRED if not stage.GetPrimAtPath(p).IsValid()]
check("required prim paths", not missing, f"missing={missing}")
check("units: metersPerUnit=1, upAxis=Z", UsdGeom.GetStageMetersPerUnit(stage) == 1.0 and UsdGeom.GetStageUpAxis(stage) == "Z",
      f"mpu={UsdGeom.GetStageMetersPerUnit(stage)} up={UsdGeom.GetStageUpAxis(stage)}")
g = UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene")).GetGravityMagnitudeAttr().Get()
check("microgravity physics scene", g == 0.0, f"gravity={g}")

bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], useExtentsHint=False)
# variant-independent geometry checks are done in the "visual" selection, then each layer's visibility is checked
vset = stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("truthLayer")
check("truth-layer variant set", sorted(vset.GetVariantNames()) == sorted(LAYERS), f"{vset.GetVariantNames()}")
vset.SetVariantSelection("visual")


def extent(path):
    r = bbc.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    return [round(v, 4) for v in (r.GetMax() - r.GetMin())]


# handrails (BBoxCache skips invisible prims -> measure collision in the collision layer)
vset.SetVariantSelection("collision")
bbc.Clear()
rails = {}
for name in ("Handrail_A", "Handrail_B"):
    grip = sorted(extent(f"/World/Environment/ISS/Collision/{name}/Grip"), reverse=True)
    full = sorted(extent(f"/World/Environment/ISS/Collision/{name}"), reverse=True)
    rails[name] = {"grip_len_m": grip[0], "grip_section_m": grip[1:], "assembly_m": full}
    # assembly = [length, standoff + grip depth from mount face, grip width]
    ok = 0.30 <= grip[0] <= 0.70 and 0.015 <= grip[1] <= 0.05 and 0.015 <= grip[2] <= 0.05 and 0.06 <= full[1] <= 0.12
    check(f"{name} dimensions plausible", ok, json.dumps(rails[name]))

# collision independence
vis_root, col_root = "/World/Environment/ISS/Visual", "/World/Environment/ISS/Collision"
col = [p for p in Usd.PrimRange(stage.GetPrimAtPath(col_root)) if p.IsA(UsdGeom.Gprim)]
vis = [p for p in Usd.PrimRange(stage.GetPrimAtPath(vis_root)) if p.IsA(UsdGeom.Gprim)]
check("collision gprims all have CollisionAPI", col and all(p.HasAPI(UsdPhysics.CollisionAPI) for p in col), f"n={len(col)}")
check("visual gprims carry no CollisionAPI", vis and not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in vis), f"n={len(vis)}")
check("collision prims are not instances/references of visual",
      not any(p.HasAuthoredReferences() or p.IsInstance() for p in col), "authored proxies")
check("collision proxy count << visual count", len(col) < len(vis) / 4, f"collision={len(col)} visual={len(vis)}")

# affordances and task references
aff = [p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Environment/ISS/Affordances")) if p.HasAttribute("iss:affordance")]
bad = []
for p in aff:
    for rel in ("iss:collisionProxy", "iss:visualSource"):
        t = p.GetRelationship(rel).GetTargets()
        if len(t) != 1 or not stage.GetPrimAtPath(t[0]).IsValid():
            bad.append(f"{p.GetPath()}:{rel}")
kinds = sorted(p.GetAttribute("iss:affordance").Get() for p in aff)
check("affordances resolve to real collision + visual prims", aff and not bad, f"kinds={kinds} bad={bad}")
check("two graspable handrails + maintenance target", kinds.count("graspable") == 2 and "maintenance_target" in kinds, str(kinds))
for path, rel in (("/World/Task/Targets/MaintenancePanel", "iss:affordance"), ("/World/Task/Waypoints/R2_Park", "iss:graspCandidates")):
    t = stage.GetPrimAtPath(path).GetRelationship(rel).GetTargets()
    check(f"{path} -> {rel} resolves", len(t) == 1 and stage.GetPrimAtPath(t[0]).IsValid(), str(t))


# R2: classification + physics disabled + conservative clearance (per-gprim world AABBs)
vset.SetVariantSelection("visual")
bbc.Clear()
r2 = stage.GetPrimAtPath("/World/Actors/R2_Visual")
cls = r2.GetAttribute("iss:classification").Get()
check("R2 classified KINEMATIC_VISUAL_ONLY", cls == "KINEMATIC_VISUAL_ONLY", str(cls))
r2_prims = list(Usd.PrimRange(r2, Usd.TraverseInstanceProxies()))
def _schemas(p):
    lop = p.GetMetadata("apiSchemas")
    return set(p.GetAppliedSchemas()) | (set(lop.ApplyOperations([])) if lop else set())


art = [f"{p.GetPath()}:{sorted(s for s in _schemas(p) if 'ArticulationRootAPI' in s)}" for p in r2_prims
       if any("ArticulationRootAPI" in s for s in _schemas(p))]
joints = [p for p in r2_prims if p.IsA(UsdPhysics.Joint)]
bodies = [p for p in r2_prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
nonkin = [str(p.GetPath()) for p in bodies if not p.GetAttribute("physics:kinematicEnabled").Get()]
check("R2 has no articulation root", not art, str(art[:3]))
check("R2 has no active joints", not joints, f"active joints={len(joints)}")
check("R2 rigid bodies all kinematic", bodies and not nonkin, f"bodies={len(bodies)} non-kinematic={nonkin[:3]}")
r2_meshes = [p for p in r2_prims if p.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(p).ComputePurpose() in ("default", "render")]
worst, worst_prim = 1e9, ""
lim_oct = 2 * HALF - CHAMFER
for p in r2_meshes:
    r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
    if r.IsEmpty():
        continue
    lo, hi = r.GetMin(), r.GetMax()
    for y in (lo[1], hi[1]):
        for z in (lo[2], hi[2]):
            margin = min(HALF - abs(y), HALF - abs(z), (lim_oct - abs(y) - abs(z)) / 2 ** 0.5)
            if margin < worst:
                worst, worst_prim = margin, str(p.GetPath())
check("R2 inside free aisle (conservative AABB clearance > 0)", worst > 0.0 and len(r2_meshes) > 20,
      f"min clearance {worst:.3f} m at {worst_prim.split('/')[-2] if worst_prim else ''}; meshes={len(r2_meshes)}")
from build_iss_microlab import X_MAX, X_MIN  # noqa: E402
xs = [bbc.ComputeWorldBound(p).ComputeAlignedRange() for p in r2_meshes]
xs = [r for r in xs if not r.IsEmpty()]
check("R2 between bulkheads", min(r.GetMin()[0] for r in xs) > X_MIN and max(r.GetMax()[0] for r in xs) < X_MAX,
      f"x=[{min(r.GetMin()[0] for r in xs):.3f}, {max(r.GetMax()[0] for r in xs):.3f}]")
vset.SetVariantSelection("collision")
bbc.Clear()
task_boxes = {n: bbc.ComputeWorldBound(stage.GetPrimAtPath(f"{col_root}/{n}")).ComputeAlignedRange()
              for n in ("Handrail_A", "Handrail_B", "MaintenancePanel")}
vset.SetVariantSelection("visual")
bbc.Clear()
hits = []
for p in r2_meshes:
    r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
    for n, b in task_boxes.items():
        if not r.IsEmpty() and not Gf.Range3d.GetIntersection(r, b).IsEmpty():
            hits.append(f"{p.GetParent().GetName()}x{n}")
check("R2 meshes do not intersect handrail/panel proxies (AABB)", not hits, f"overlaps={hits[:4]}")
r2box = bbc.ComputeWorldBound(r2).ComputeAlignedRange()
check("R2 overall extent plausible (0.8-2.6 m)", 0.8 < max(r2box.GetMax() - r2box.GetMin()) < 2.6,
      f"extent={[round(v, 3) for v in (r2box.GetMax() - r2box.GetMin())]}")

# camera / lighting
camp = stage.GetPrimAtPath("/World/Cameras/Hero")
eye = UsdGeom.Xformable(camp).ComputeLocalToWorldTransform(0).ExtractTranslation()
check("hero camera resolves and is inside the bay", camp.IsA(UsdGeom.Camera) and abs(eye[1]) < HALF and abs(eye[2]) < HALF,
      f"eye={[round(v, 3) for v in eye]} focal={UsdGeom.Camera(camp).GetFocalLengthAttr().Get()}mm")
lights = [p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Environment/ISS/Lighting")) if p.HasAPI(UsdLux.LightAPI)]
check("lighting resolves", len(lights) >= 3, f"lights={len(lights)}: {sorted({p.GetTypeName() for p in lights})}")

# per-layer visibility
for name, flags in LAYERS.items():
    vset.SetVariantSelection(name)
    vis_state = [UsdGeom.Imageable(stage.GetPrimAtPath(p)).ComputeVisibility() != "invisible" for p in LAYER_PRIMS]
    check(f"layer '{name}' visibility", vis_state == list(flags), f"{dict(zip(LAYER_PRIMS, vis_state))}")
vset.SetVariantSelection("visual")

# the physics track is untouched by this slice
gate = subprocess.run(["git", "diff", "--quiet", "HEAD", "--", "configs/", "outputs/r2_derived"], cwd=ROOT).returncode
check("momentum gate config unchanged vs HEAD", gate == 0, "git diff HEAD -- configs/")

out = ROOT / "outputs" / "iss_microlab_v0"
out.mkdir(parents=True, exist_ok=True)
report = {"scene": str(SCENE.relative_to(ROOT)), "mode": "isaac" if args.isaac else "pxr", "handrails": rails,
          "passed": sum(r["pass"] for r in results), "total": len(results), "results": results}
(out / f"scene_check{'_isaac' if args.isaac else ''}.json").write_text(json.dumps(report, indent=1), encoding="utf-8")
for r in results:
    print(("PASS " if r["pass"] else "FAIL ") + r["check"] + (f"  [{r['detail']}]" if r["detail"] else ""))
print(f"{report['passed']}/{report['total']} checks passed")
code = 0 if report["passed"] == report["total"] else 1
if args.isaac:
    app.close()
sys.exit(code)

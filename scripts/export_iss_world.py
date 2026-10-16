"""Export the full-ISS collision world and task anchors (scene frame) for external planners such as cuRobo.

C:\\isaacsim\\python.bat scripts/export_iss_world.py
Reads scenes/iss_full_v0.usda (composed, collision layer) and writes outputs/iss_full_v0/world/:
  collision_world.npz  per-collider triangles in the scene frame (module triangle meshes, US Lab proxy boxes,
                       authored passage walls), keyed by prim path
  anchors.json         graspable rails, hatch openings (centre, axis, opening), passages, maintenance target,
                       each with its source prim and provenance
The exporter only reads the scene; nothing is authored.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom

import build_iss_full as B

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_full_v0.usda"
OUT = ROOT / "outputs" / "iss_full_v0" / "world"
CUBE_TRIS = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                      [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
CUBE_PTS = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])


def world_matrix(prim):
    return np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=np.float64)


def cube_tris(prim):
    size = UsdGeom.Cube(prim).GetSizeAttr().Get() or 2.0
    p = np.c_[CUBE_PTS * size, np.ones(8)] @ world_matrix(prim)
    return p[:, :3][CUBE_TRIS]


def box_frame(prim):
    m = world_matrix(prim)
    size = UsdGeom.Cube(prim).GetSizeAttr().Get() or 2.0
    axes = m[:3, :3] * size
    return m[3, :3], axes      # centre, rows = scaled local axes


def main():
    stage = Usd.Stage.Open(str(SCENE))
    stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection("affordance")
    col_root = f"{B.ISS}/Collision"
    colliders, kinds = {}, {}
    for p in Usd.PrimRange(stage.GetPrimAtPath(col_root)):
        if p.IsA(UsdGeom.Mesh):
            colliders[str(p.GetPath())] = B.world_triangles(stage, str(p.GetPath())).astype(np.float32)
            kinds[str(p.GetPath())] = "triangle_mesh"
        elif p.IsA(UsdGeom.Cube):
            colliders[str(p.GetPath())] = cube_tris(p).astype(np.float32)
            kinds[str(p.GetPath())] = "authored_passage_box" if "/Passages/" in str(p.GetPath()) else "proxy_box"
    OUT.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT / "collision_world.npz", **{k.replace("/", "|"): v for k, v in colliders.items()})

    anchors = {"frame": "scene (Z = zenith); see build_iss_full.py", "rails": [], "hatches": [], "passages": [],
               "targets": []}
    for p in Usd.PrimRange(stage.GetPrimAtPath(f"{B.ISS}/Affordances")):
        kind = p.GetAttribute("iss:affordance").Get() if p.HasAttribute("iss:affordance") else None
        if not kind:
            continue
        c, axes = box_frame(p)
        lengths = np.linalg.norm(axes, axis=1)
        rec = {"prim": str(p.GetPath()), "centre_m": c.round(5).tolist(),
               "axes_unit": (axes / lengths[:, None]).round(6).tolist(), "extent_m": lengths.round(5).tolist(),
               "provenance": p.GetCustomDataByKey("provenance") or "scene affordance"}
        if kind == "graspable":
            long_ax = int(np.argmax(lengths))
            rec["grip_axis_unit"] = (axes[long_ax] / lengths[long_ax]).round(6).tolist()
            rec["grip_half_length_m"] = round(float(lengths[long_ax] / 2), 5)
            anchors["rails"].append(rec)
        elif kind == "hatch":
            rec["opening_m"] = list(p.GetAttribute("iss:hatchClearOpening_m").Get())
            rec["modules"] = [str(t) for t in p.GetRelationship("iss:modules").GetTargets()]
            anchors["hatches"].append(rec)
        elif kind == "passage":
            rec["status"] = p.GetAttribute("iss:status").Get()
            anchors["passages"].append(rec)
        else:
            anchors["targets"].append(dict(rec, kind=kind))
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    stage.GetPrimAtPath("/World").GetVariantSets().GetVariantSet("truthLayer").SetVariantSelection("visual")
    anchors["modules"] = {}
    for name in B.MODULES:
        r = bbc.ComputeWorldBound(stage.GetPrimAtPath(f"{B.ISS}/Visual/{name}")).ComputeAlignedRange()
        anchors["modules"][name] = {"bbox_min_m": [round(v, 4) for v in r.GetMin()],
                                    "bbox_max_m": [round(v, 4) for v in r.GetMax()]}
    anchors["scene_sha256"] = hashlib.sha256(SCENE.read_bytes()).hexdigest()
    anchors["colliders"] = {k: {"kind": kinds[k], "triangles": int(len(v))} for k, v in colliders.items()}
    (OUT / "anchors.json").write_text(json.dumps(anchors, indent=1) + "\n", encoding="utf-8")
    print(json.dumps({"colliders": len(colliders), "triangles": int(sum(len(v) for v in colliders.values())),
                      "rails": len(anchors["rails"]), "hatches": len(anchors["hatches"]),
                      "passages": len(anchors["passages"]), "targets": len(anchors["targets"])}))


if __name__ == "__main__":
    main()

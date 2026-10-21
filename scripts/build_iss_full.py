"""Build the full-ISS-interior scene v0 (USOS modules available in nasa/astrobee_media) on the S1 scene contract.

Pure pxr + numpy authoring; run with C:\\isaacsim\\python.bat scripts/build_iss_full.py
Visual      = the 7 NASA interior meshes converted by scripts/import_iss_media.py, posed exactly as the <visual>
              origins in astrobee_iss/urdf/model.urdf (Astrobee ISS world frame), + the two S1 US Lab handrails.
Collision   = US Lab: the S1 primitive proxies (measured aisle), carried over rigidly with the US Lab mesh, without
              the S1 hatch caps; every other module: static triangle mesh of its interior mesh (approximation "none",
              no SDF). Strategy recorded per module (iss:collisionStrategy).
Affordances = the S1 handrails + maintenance target (US Lab only; no public handrail layout exists for the other
              modules, so none are invented) + one 'hatch' affordance per measured inter-module hatch.
Scene frame = Astrobee ISS world frame (x forward, y starboard, z nadir) rotated 180 deg about x, so +Z = zenith.
Not present in the source and therefore absent here: PMM, Quest airlock, JEM ELM-PS (logistics), Russian segment.
"""
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

import build_iss_microlab as S1

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_full_v0.usda"
MEDIA = ROOT / "assets" / "cache" / "iss_media"
URDF = ROOT / "assets" / "raw" / "astrobee_media" / "astrobee_iss" / "urdf" / "model.urdf"
OUT = ROOT / "outputs" / "iss_full_v0"
ISS = "/World/Environment/ISS"
STATION_TO_SCENE_RX_DEG = 180.0
MODULES = {  # scene name: (mesh, ISS name)
    "Node1": ("node_1", "Node 1 Unity"), "USLab": ("us_lab", "US Lab Destiny"), "Node2": ("node_2", "Node 2 Harmony"),
    "Columbus": ("eu_lab", "Columbus (ESA)"), "JEM_PM": ("jpm", "JEM Pressurized Module Kibo"),
    "Node3": ("node_3", "Node 3 Tranquility"), "Cupola": ("cupola", "Cupola"),
}
# berthing topology of the modules present (ISS configuration; poses come only from the URDF)
HATCHES = [("Node1", "USLab"), ("USLab", "Node2"), ("Node2", "Columbus"), ("Node2", "JEM_PM"), ("Node1", "Node3"),
           ("Node3", "Cupola")]
KNOWN_GAPS = ["PMM Leonardo", "Quest airlock", "JEM ELM-PS logistics module", "Russian segment (FGB/SM/MRM/MLM)",
              "PMA-2/PMA-3 and visiting vehicles", "ISS handrail layout outside the US Lab (no public source)"]
# alignment tolerances, fixed before measuring: hatch centres of adjacent modules, and gap along the hatch axis
HATCH_TOL = {"centre_offset_m": 0.05, "axial_gap_m": 0.10}
HATCH_SLAB_M, HATCH_WINDOW_M, HATCH_PX_M = 0.50, 1.80, 0.01
LIGHT = {"intensity": 9000.0, "radius": 0.10, "kelvin": 4800.0, "spacing_m": 1.6}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def urdf_poses():
    """Visual and collision <origin> plus mesh scale for every module mesh in the Astrobee ISS URDF."""
    body = ET.parse(URDF).getroot().find("link")
    poses = {}
    for kind in ("visual", "collision"):
        for node in body.findall(kind):
            mesh = node.find("geometry/mesh")
            name = Path(mesh.get("filename")).stem
            o = node.find("origin")
            poses.setdefault(name, {})[kind] = {
                "xyz": [float(v) for v in o.get("xyz").split()], "rpy": [float(v) for v in o.get("rpy").split()],
                "scale": [float(v) for v in mesh.get("scale", "1 1 1").split()]}
    return poses


def pose_ops(prim, pose):
    """URDF origin (rpy fixed-axis = Rz*Ry*Rx = USD rotateXYZ) followed by the mesh scale."""
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*pose["xyz"]))
    x.AddRotateXYZOp().Set(Gf.Vec3f(*[math.degrees(v) for v in pose["rpy"]]))
    if pose["scale"] != [1.0, 1.0, 1.0]:
        x.AddScaleOp().Set(Gf.Vec3f(*pose["scale"]))


def s1_uslab_map(stage, uslab_prim):
    """Row-vector matrix taking S1 micro-lab coordinates to this scene's ISS frame for the same US Lab mesh.

    S1 posed the converted us_lab.usd with translate (4.2,0,0), rotateX 90, scale 1.0858 (it assumed a Y-up
    conversion; the converted file keeps the DAE's Z-up coordinates). Carrying every S1 US Lab element through
    S1^-1 * URDF keeps rails/proxies on the same mesh surfaces; it does not make S1's wall names station directions.
    """
    tmp_stage = Usd.Stage.CreateInMemory()   # keep a reference: the prim dies with its stage
    tmp = tmp_stage.DefinePrim("/S1", "Xform")
    S1._place(tmp, S1.USLAB_XFORM["translate"], (S1.USLAB_XFORM["scale"],) * 3, S1.USLAB_XFORM["rotateXYZ"])
    m_s1 = UsdGeom.Xformable(tmp).GetLocalTransformation()
    m_full = UsdGeom.Xformable(uslab_prim).GetLocalTransformation()
    return m_s1.GetInverse() * m_full


# ---- geometry helpers (numpy; used by the builder and by scripts/check_iss_full.py) ------------------------
def world_triangles(stage, root):
    """All triangles (N, 3, 3) of the visible render meshes under root, in world coordinates."""
    cache = UsdGeom.XformCache()
    tris = []
    for p in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = np.asarray(m.GetPointsAttr().Get(), dtype=np.float64)
        counts = np.asarray(m.GetFaceVertexCountsAttr().Get())
        idx = np.asarray(m.GetFaceVertexIndicesAttr().Get())
        if pts.size == 0 or counts.size == 0:
            continue
        mat = np.asarray(cache.GetLocalToWorldTransform(p), dtype=np.float64)
        w = np.c_[pts, np.ones(len(pts))] @ mat
        w = w[:, :3]
        starts = np.r_[0, np.cumsum(counts)[:-1]]
        for k in range(3, int(counts.max()) + 1):   # fan-triangulate every polygon size present
            sel = starts[counts == k]
            for j in range(1, k - 1):
                tris.append(np.stack([w[idx[sel]], w[idx[sel + j]], w[idx[sel + j + 1]]], axis=1))
    return np.concatenate(tris) if tris else np.zeros((0, 3, 3))


def principal_axis(v):
    a = int(np.argmax(np.abs(v)))
    d = np.zeros(3)
    d[a] = math.copysign(1.0, v[a])
    return d


def hatch_opening(tris, d, centre, near_end):
    """Measure one module's hatch opening facing direction d.

    Takes the module's triangles within HATCH_SLAB_M of its extreme along d (near_end='max') or -d ('min'), restricted
    to a square window of half-size HATCH_WINDOW_M around `centre` in the plane normal to d, rasterises their
    projection, and returns the largest open region fully enclosed by geometry (line of sight through the slab).
    """
    a = int(np.argmax(np.abs(d)))
    u, v = [i for i in range(3) if i != a]
    lat = np.max(np.abs(tris[:, :, [u, v]] - centre[[u, v]]), axis=(1, 2)) <= HATCH_WINDOW_M
    t = tris[lat]
    if not len(t):
        return None
    s = t[:, :, a]
    end = s.max() if near_end == "max" else s.min()
    keep = (s.max(axis=1) >= end - HATCH_SLAB_M) if near_end == "max" else (s.min(axis=1) <= end + HATCH_SLAB_M)
    t = t[keep]
    from PIL import Image, ImageDraw
    from scipy import ndimage
    n = int(round(2 * HATCH_WINDOW_M / HATCH_PX_M))
    img = Image.new("1", (n, n), 0)
    draw = ImageDraw.Draw(img)
    o = centre[[u, v]] - HATCH_WINDOW_M
    for tri in t:
        draw.polygon([tuple(x) for x in ((tri[:, [u, v]] - o) / HATCH_PX_M)], fill=1, outline=1)
    covered = np.asarray(img, dtype=bool).T          # [iu, iv]
    lab, count = ndimage.label(~covered)
    border = set(np.unique(np.r_[lab[0], lab[-1], lab[:, 0], lab[:, -1]]))
    best = None
    for k in range(1, count + 1):
        if k in border:
            continue
        area = int((lab == k).sum())
        if best is None or area > best[1]:
            best = (k, area)
    if best is None:
        return {"status": "NO_ENCLOSED_OPENING", "end_m": float(end), "triangles_in_slab": int(len(t))}
    iu, iv = np.nonzero(lab == best[0])
    cu, cv = o + (np.array([iu.mean(), iv.mean()]) + 0.5) * HATCH_PX_M
    c = np.zeros(3)
    c[a], c[u], c[v] = end, cu, cv
    return {"status": "MEASURED", "centre_m": c.tolist(), "end_m": float(end), "axis": int(a),
            "size_m": [round(float((iu.max() - iu.min() + 1) * HATCH_PX_M), 3),
                       round(float((iv.max() - iv.min() + 1) * HATCH_PX_M), 3)],
            "open_area_m2": round(best[1] * HATCH_PX_M ** 2, 4), "triangles_in_slab": int(len(t))}


def measure_hatches(stage, visual_root, shift=None):
    """Hatch alignment for every berthing pair; shift = {module: world offset} for what-if comparisons."""
    tris, boxes = {}, {}
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    for name in MODULES:
        t = world_triangles(stage, f"{visual_root}/{name}")
        off = np.asarray((shift or {}).get(name, (0, 0, 0)), dtype=np.float64)
        tris[name] = t + off
        r = bbc.ComputeWorldBound(stage.GetPrimAtPath(f"{visual_root}/{name}")).ComputeAlignedRange()
        boxes[name] = (np.asarray(r.GetMin()) + off, np.asarray(r.GetMax()) + off)
    out = []
    for a_name, b_name in HATCHES:
        ca, cb = (sum(boxes[a_name]) / 2), (sum(boxes[b_name]) / 2)
        d = principal_axis(cb - ca)
        axis = int(np.argmax(np.abs(d)))
        mid = (ca + cb) / 2
        A = hatch_opening(tris[a_name], d, mid, "max" if d[axis] > 0 else "min")
        B = hatch_opening(tris[b_name], d, mid, "min" if d[axis] > 0 else "max")
        row = {"pair": f"{a_name}-{b_name}", "axis": "xyz"[axis], "direction_sign": int(d[axis]), "a": A, "b": B}
        if A and B and A["status"] == B["status"] == "MEASURED":
            pa, pb = np.asarray(A["centre_m"]), np.asarray(B["centre_m"])
            lat = [i for i in range(3) if i != axis]
            row["centre_offset_m"] = round(float(np.linalg.norm(pa[lat] - pb[lat])), 4)
            row["axial_gap_m"] = round(float((pb[axis] - pa[axis]) * d[axis]), 4)   # + gap, - overlap
            row["within_tolerance"] = (row["centre_offset_m"] <= HATCH_TOL["centre_offset_m"]
                                       and abs(row["axial_gap_m"]) <= HATCH_TOL["axial_gap_m"])
            row["centre_m"] = ((pa + pb) / 2).round(4).tolist()
        else:
            row["within_tolerance"] = None
        out.append(row)
    return out


# ---- layers --------------------------------------------------------------------------------------------
def build_visual(stage, poses):
    root = f"{ISS}/Visual"
    UsdGeom.Xform.Define(stage, root)
    prims = {}
    for name, (mesh, label) in MODULES.items():
        p = stage.DefinePrim(f"{root}/{name}", "Xform")
        p.GetReferences().AddReference(S1.rel_asset(MEDIA / f"{mesh}.usd"))
        pose_ops(p, poses[mesh]["visual"])
        p.CreateAttribute("iss:module", Sdf.ValueTypeNames.String).Set(label)
        p.CreateAttribute("iss:sourceMesh", Sdf.ValueTypeNames.String).Set(
            f"nasa/astrobee_media@1cb0620 astrobee_iss/meshes/{mesh}.dae")
        p.CreateAttribute("iss:poseSource", Sdf.ValueTypeNames.String).Set(
            "astrobee_iss/urdf/model.urdf <visual> origin" + (" (collision origin differs; see iss:collisionOriginXyz)"
                                                               if poses[mesh]["collision"]["xyz"] != poses[mesh]["visual"]["xyz"] else ""))
        if poses[mesh]["collision"]["xyz"] != poses[mesh]["visual"]["xyz"]:
            p.CreateAttribute("iss:collisionOriginXyz", Sdf.ValueTypeNames.Double3).Set(Gf.Vec3d(*poses[mesh]["collision"]["xyz"]))
            p.CreateAttribute("iss:originStatus", Sdf.ValueTypeNames.String).Set(
                "UNKNOWN which origin is correct: URDF visual and collision origins differ; visual origin used")
        prims[name] = p
    # S1 handrails, carried with the US Lab
    local = UsdGeom.Xform.Define(stage, f"{root}/USLab_S1Frame")
    local.AddTransformOp().Set(s1_uslab_map(stage, prims["USLab"]))
    for name, spec in S1.LAYOUT["handrails"].items():
        h = stage.DefinePrim(f"{root}/USLab_S1Frame/{name}", "Xform")
        h.GetReferences().AddReference(S1.rel_asset(MEDIA / "handrail_30.usd"))
        x = UsdGeom.Xformable(h)
        x.ClearXformOpOrder()
        x.AddTransformOp().Set(S1.handrail_mount(spec["wall"], spec["x"], spec["u"]))
    return prims


def build_collision(stage, poses, uslab_prim):
    root = f"{ISS}/Collision"
    UsdGeom.Xform.Define(stage, root)
    strategy = {}
    # US Lab: S1 proxies (walls, corner standoffs, bulkheads with hatch, hatch tunnels, rails, panel), no hatch caps
    lab = UsdGeom.Xform.Define(stage, f"{root}/USLab")
    lab.AddTransformOp().Set(s1_uslab_map(stage, uslab_prim))
    L, t, X, HY, HZ, HH = 2 * S1.X_HALF, 0.05, S1.X_HALF, S1.HY, S1.HZ, S1.HATCH_HALF
    slabs = {"WallA": ((0, HY + t / 2, 0), (L, t, 2 * HZ)), "WallB": ((0, -HY - t / 2, 0), (L, t, 2 * HZ)),
             "WallC": ((0, 0, HZ + t / 2), (L, 2 * HY, t)), "WallD": ((0, 0, -HZ - t / 2), (L, 2 * HY, t))}
    for end, sx in (("Aft", -1), ("Fwd", +1)):
        x0 = sx * (X + t / 2)
        for k, (cy, cz, ly, lz) in enumerate((
                (0, (HZ + HH) / 2, 2 * HY, HZ - HH), (0, -(HZ + HH) / 2, 2 * HY, HZ - HH),
                ((HY + HH) / 2, 0, HY - HH, 2 * HH), (-(HY + HH) / 2, 0, HY - HH, 2 * HH))):
            slabs[f"{end}Bulkhead{k}"] = ((x0, cy, cz), (t, ly, lz))
        tunnel = 4.2 - X
        for k, (cy, cz, ly, lz) in enumerate(((0, HH + 0.01, 2 * HH, 0.02), (0, -HH - 0.01, 2 * HH, 0.02),
                                              (HH + 0.01, 0, 0.02, 2 * HH), (-HH - 0.01, 0, 0.02, 2 * HH))):
            slabs[f"{end}Hatchway{k}"] = ((sx * (X + tunnel / 2), cy, cz), (tunnel, ly, lz))
    for name, (c, s) in slabs.items():
        S1.box(stage, f"{root}/USLab/{name}", c, s)
    for sy in (+1, -1):
        for sz in (+1, -1):
            (cy, cz), th, width = S1.corner_geom(sy, sz, t)
            S1.box(stage, f"{root}/USLab/Standoff_{'A' if sy > 0 else 'B'}{'C' if sz > 0 else 'D'}", (0, cy, cz),
                   (L, width, t), None, (th, 0, 0))
    for name, spec in S1.LAYOUT["handrails"].items():
        S1.handrail_proxy(stage, f"{root}/USLab", name, spec["wall"], spec["x"], spec["u"])
    tp = S1.LAYOUT["panel"]
    S1.box(stage, f"{root}/USLab/MaintenancePanel", S1.wall_point(tp["wall"], tp["x"], tp["u"], -0.01),
           S1.wall_size(tp["wall"], tp["sx"], tp["su"], 0.02))
    for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{root}/USLab")):
        if prim.IsA(UsdGeom.Gprim):
            UsdPhysics.CollisionAPI.Apply(prim)
    strategy["USLab"] = ("primitive_proxies_S1: authored boxes from the measured S1 aisle (corner value applied to "
                         "all four corners); S1 hatch caps removed so both hatchways are open")
    # other modules: static triangle mesh of the interior mesh
    for name, (mesh, _) in MODULES.items():
        if name == "USLab":
            continue
        p = stage.DefinePrim(f"{root}/{name}", "Xform")
        p.GetReferences().AddReference(S1.rel_asset(MEDIA / f"{mesh}.usd"))
        pose_ops(p, poses[mesh]["visual"])
        n = 0
        for m in Usd.PrimRange(p):
            if m.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(m)
                UsdPhysics.MeshCollisionAPI.Apply(m).CreateApproximationAttr(UsdPhysics.Tokens.none)
                n += 1
        strategy[name] = (f"static_triangle_mesh: {n} mesh prim(s) of the interior visual mesh, approximation none "
                          "(no SDF, no convex decomposition); same pose as visual; collision fidelity UNVALIDATED")
    for name, s in strategy.items():
        stage.GetPrimAtPath(f"{root}/{name}").CreateAttribute("iss:collisionStrategy", Sdf.ValueTypeNames.String).Set(s)
    return strategy


def build_affordances(stage, M, uslab_prim, hatches):
    root = f"{ISS}/Affordances"
    UsdGeom.Xform.Define(stage, root)
    local = UsdGeom.Xform.Define(stage, f"{root}/USLab_S1Frame")
    local.AddTransformOp().Set(s1_uslab_map(stage, uslab_prim))
    lr = f"{root}/USLab_S1Frame"
    h, pad = S1.HANDRAIL, 0.004
    for name, spec in S1.LAYOUT["handrails"].items():
        g = S1.box(stage, f"{lr}/{name}_Graspable",
                   S1.wall_point(spec["wall"], spec["x"], spec["u"], -(h["standoff"] + h["grip_t"] / 2)),
                   S1.wall_size(spec["wall"], h["grip_len"] - 0.06 + pad, h["grip_w"] + pad, h["grip_t"] + pad), M["grasp"]).GetPrim()
        g.CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set("graspable")
        g.CreateAttribute("iss:graspLength_m", Sdf.ValueTypeNames.Float).Set(h["grip_len"] - 0.06)
        g.CreateRelationship("iss:collisionProxy").SetTargets([Sdf.Path(f"{ISS}/Collision/USLab/{name}/Grip")])
        g.CreateRelationship("iss:visualSource").SetTargets([Sdf.Path(f"{ISS}/Visual/USLab_S1Frame/{name}")])
    tp = S1.LAYOUT["panel"]
    t = S1.box(stage, f"{lr}/MaintenancePanel_Target", S1.wall_point(tp["wall"], tp["x"], tp["u"], -0.022),
               S1.wall_size(tp["wall"], tp["sx"] + 0.01, tp["su"] + 0.01, 0.004), M["target"]).GetPrim()
    t.CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set("maintenance_target")
    t.CreateRelationship("iss:collisionProxy").SetTargets([Sdf.Path(f"{ISS}/Collision/USLab/MaintenancePanel")])
    t.CreateRelationship("iss:visualSource").SetTargets([Sdf.Path(f"{ISS}/Visual/USLab")])
    UsdGeom.Xform.Define(stage, "/World/Task/Targets")
    tgt = UsdGeom.Xform.Define(stage, "/World/Task/Targets/MaintenancePanel").GetPrim()
    tgt.CreateRelationship("iss:affordance").SetTargets([t.GetPath()])
    # hatch affordances: thin plates at the measured hatch centres (scene frame; ISS root transform undone)
    to_station = np.asarray(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), STATION_TO_SCENE_RX_DEG)).GetInverse())
    for row in hatches:
        if not row.get("centre_m"):
            continue
        a_name, b_name = row["pair"].split("-")
        size = [min(row["a"]["size_m"][i], row["b"]["size_m"][i]) for i in range(2)]
        c = np.r_[row["centre_m"], 1.0] @ to_station
        dims = [0.0, 0.0, 0.0]
        lat = [i for i in range(3) if "xyz"[i] != row["axis"]]
        dims["xyz".index(row["axis"])] = 0.01
        dims[lat[0]], dims[lat[1]] = size
        p = S1.box(stage, f"{root}/Hatch_{a_name}_{b_name}", tuple(c[:3]), tuple(dims), M["waypoint"]).GetPrim()
        p.CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set("hatch")
        p.CreateAttribute("iss:hatchClearOpening_m", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*size))
        p.CreateAttribute("iss:measurement", Sdf.ValueTypeNames.String).Set(
            f"enclosed line-of-sight opening of both module meshes, {HATCH_SLAB_M} m slab, {HATCH_PX_M} m raster; "
            f"centre offset {row['centre_offset_m']} m, axial gap {row['axial_gap_m']} m")
        p.CreateRelationship("iss:modules").SetTargets([Sdf.Path(f"{ISS}/Visual/{a_name}"), Sdf.Path(f"{ISS}/Visual/{b_name}")])
        p.CreateRelationship("iss:collisionProxy").SetTargets([Sdf.Path(f"{ISS}/Collision/{b_name}")])
        p.CreateRelationship("iss:visualSource").SetTargets([Sdf.Path(f"{ISS}/Visual/{b_name}")])


def build_lighting(stage, uslab_prim, boxes):
    """US Lab: the S1 GLA strips (carried with the mesh). Other modules: presentation sphere lights on the
    module centre line - authored for rendering, not an ISS lighting specification."""
    root = f"{ISS}/Lighting"
    UsdGeom.Xform.Define(stage, root)
    lab = UsdGeom.Xform.Define(stage, f"{root}/USLab_S1Frame")
    lab.AddTransformOp().Set(s1_uslab_map(stage, uslab_prim))
    for sy in (+1, -1):
        (cy, cz), th, _ = S1.corner_geom(sy, +1, -0.06)
        for k, x in enumerate((-2.4, -0.8, 0.8, 2.4)):
            lt = UsdLux.RectLight.Define(stage, f"{root}/USLab_S1Frame/GLA_{'A' if sy > 0 else 'B'}{k}")
            lt.CreateWidthAttr(1.0)
            lt.CreateHeightAttr(0.12)
            lt.CreateIntensityAttr(S1.LIGHT["gla_intensity"])
            lt.CreateEnableColorTemperatureAttr(True)
            lt.CreateColorTemperatureAttr(S1.LIGHT["gla_kelvin"])
            S1._place(lt.GetPrim(), (x, cy, cz), (1, 1, 1), (th, 0, 0))
    for name, (lo, hi) in boxes.items():
        if name == "USLab":
            continue
        c, ext = (lo + hi) / 2, hi - lo
        a = int(np.argmax(ext))
        n = max(1, int(round(ext[a] / LIGHT["spacing_m"])) - 1)
        for k in range(n):
            p = c.copy()
            p[a] = lo[a] + ext[a] * (k + 1) / (n + 1)
            lt = UsdLux.SphereLight.Define(stage, f"{root}/{name}_{k}")
            lt.CreateRadiusAttr(LIGHT["radius"])
            lt.CreateIntensityAttr(LIGHT["intensity"])
            lt.CreateEnableColorTemperatureAttr(True)
            lt.CreateColorTemperatureAttr(LIGHT["kelvin"])
            lt.CreateTreatAsPointAttr(False)
            UsdGeom.Xformable(lt).AddTranslateOp().Set(Gf.Vec3d(*to_station_point(p)))


def to_station_point(p_scene):
    m = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), STATION_TO_SCENE_RX_DEG)).GetInverse()
    return m.Transform(Gf.Vec3d(*[float(v) for v in p_scene]))


def build_cameras(stage, boxes, hatches):
    """One wide camera per module (inside, at a measured hatch, looking through the module centre) + one view
    from the US Lab through its forward hatch into Node 2. All in the scene frame (Z = zenith)."""
    UsdGeom.Xform.Define(stage, "/World/Cameras")
    cams = {}
    by_module = {}
    for row in hatches:
        if row.get("centre_m"):
            for name in row["pair"].split("-"):
                by_module.setdefault(name, []).append(np.asarray(row["centre_m"]))
    for name, (lo, hi) in boxes.items():
        c = (lo + hi) / 2
        if name == "USLab" and len(by_module.get(name, [])) == 2:
            c = sum(by_module[name]) / 2            # mesh bbox is asymmetric: use the axis between both hatches
        h = by_module.get(name, [None])[0]
        if h is None:
            ext = hi - lo
            a = int(np.argmax(ext))
            h = c.copy()
            h[a] = lo[a]
        d = c - h
        eye = h + d * (0.30 if np.linalg.norm(d) > 1.5 else 0.25)
        target = c + d
        cams[f"{name}_Wide"] = (eye, target, 14.0)
    if len(by_module.get("USLab", [])) == 2:
        hatch = [r for r in hatches if r["pair"] == "USLab-Node2"][0]
        hc = np.asarray(hatch["centre_m"])
        axis = "xyz".index(hatch["axis"])
        d = np.zeros(3)
        d[axis] = hatch["direction_sign"]
        lab_axis = sum(by_module["USLab"]) / 2
        eye = hc - d * 2.6
        eye[[i for i in range(3) if i != axis]] = lab_axis[[i for i in range(3) if i != axis]]
        cams["USLab_to_Node2_ThroughHatch"] = (eye, hc + d * 3.0, 18.0)
    for name, (eye, target, focal) in cams.items():
        cam = UsdGeom.Camera.Define(stage, f"/World/Cameras/{name}")
        cam.CreateFocalLengthAttr(focal)
        cam.CreateHorizontalApertureAttr(36.0)
        cam.CreateVerticalApertureAttr(36.0 * 9 / 16)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 80.0))
        cam.CreateFStopAttr(0.0)
        x = UsdGeom.Xformable(cam.GetPrim())
        x.ClearXformOpOrder()
        up = (0, 0, 1) if abs((target - eye)[2]) < 0.9 * np.linalg.norm(target - eye) else (1, 0, 0)   # vertical view
        x.AddTransformOp().Set(S1.look_at_xform(tuple(eye), tuple(target), up))
    return {k: {"eye": [round(float(v), 3) for v in e], "target": [round(float(v), 3) for v in t], "focal_mm": f}
            for k, (e, t, f) in cams.items()}


PASSAGE_PROVENANCE = ("AUTHORED_PROXY: CBM vestibule missing from source mesh; dimensions from measured hatch "
                      "openings, not NASA drawings")
PASSAGE_WALL_M = 0.03


def build_passages(stage, M, hatches):
    """Square-tube vestibule proxies spanning each measured hatch gap (Visual walls, Collision walls, affordance).

    Inner opening = the smaller measured opening of the two modules per lateral axis; centred between the two
    measured hatch centres; length = from the first module's hatch end plane to the second's. Module poses are not
    touched. Every prim carries customData provenance = PASSAGE_PROVENANCE.
    """
    to_station = np.asarray(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), STATION_TO_SCENE_RX_DEG)).GetInverse())
    mat = S1.material(stage, "/World/Looks/AuthoredProxyPassage", (0.55, 0.60, 0.68), 0.7)
    out = []
    for row in hatches:
        if not row.get("centre_m") or row["axial_gap_m"] <= 0:
            continue
        a_name, b_name = row["pair"].split("-")
        ax = "xyz".index(row["axis"])
        lat = [i for i in range(3) if i != ax]
        inner = [min(row["a"]["size_m"][k], row["b"]["size_m"][k]) for k in range(2)]
        s0, s1 = sorted((row["a"]["end_m"], row["b"]["end_m"]))
        c = np.asarray(row["centre_m"], dtype=float)
        c[ax] = (s0 + s1) / 2
        unknown = row["centre_offset_m"] > HATCH_TOL["centre_offset_m"] or max(row["b"]["size_m"]) > 1.5
        name = f"Passage_{a_name}_{b_name}"
        walls = []
        for k in range(2):            # two walls normal to each lateral axis
            for sgn in (-1, 1):
                wc, ws = c.copy(), [0.0, 0.0, 0.0]
                ws[ax] = s1 - s0
                ws[lat[k]] = PASSAGE_WALL_M
                ws[lat[1 - k]] = inner[1 - k] + 2 * PASSAGE_WALL_M
                wc[lat[k]] += sgn * (inner[k] / 2 + PASSAGE_WALL_M / 2)
                walls.append((f"Wall{'uv'[k]}{'n' if sgn < 0 else 'p'}", (np.r_[wc, 1.0] @ to_station)[:3], ws))
        for layer in ("Visual", "Collision"):
            root = UsdGeom.Xform.Define(stage, f"{ISS}/{layer}/Passages/{name}").GetPrim()
            for wname, wc, ws in walls:
                g = S1.box(stage, f"{root.GetPath()}/{wname}", tuple(wc), tuple(ws), mat if layer == "Visual" else None).GetPrim()
                if layer == "Collision":
                    UsdPhysics.CollisionAPI.Apply(g)
                g.SetCustomDataByKey("provenance", PASSAGE_PROVENANCE)
            root.SetCustomDataByKey("provenance", PASSAGE_PROVENANCE)
            if layer == "Collision":
                root.CreateAttribute("iss:collisionStrategy", Sdf.ValueTypeNames.String).Set(
                    "authored_proxy: four primitive box walls (square tube), not from source mesh")
        m = S1.box(stage, f"{ISS}/Affordances/{name}", tuple((np.r_[c, 1.0] @ to_station)[:3]),
                   (0.05, 0.05, 0.05), M["waypoint"]).GetPrim()   # marker at the passage centre
        m.CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set("passage")
        m.CreateAttribute("iss:passageInnerOpening_m", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*inner))
        m.CreateAttribute("iss:passageLength_m", Sdf.ValueTypeNames.Float).Set(float(s1 - s0))
        m.CreateAttribute("iss:status", Sdf.ValueTypeNames.String).Set(
            "UNKNOWN: hatch centres offset beyond tolerance and/or the second opening may be a window, not a hatch"
            if unknown else "AUTHORED_PROXY")
        m.CreateRelationship("iss:collisionProxy").SetTargets([Sdf.Path(f"{ISS}/Collision/Passages/{name}")])
        m.CreateRelationship("iss:visualSource").SetTargets([Sdf.Path(f"{ISS}/Visual/Passages/{name}")])
        m.SetCustomDataByKey("provenance", PASSAGE_PROVENANCE)
        out.append({"pair": row["pair"], "inner_opening_m": inner, "length_m": round(float(s1 - s0), 4),
                    "centre_scene_m": c.round(4).tolist(), "status": "UNKNOWN" if unknown else "AUTHORED_PROXY"})
    return out


ROUTE_SPACING_M, ROUTE_STANDOFF_M = 0.50, 0.22   # 0.08/0.15 m: G1 hand spheres, then the wrist, hit the racks
ROUTE_MIN_CLEARANCE_M = 0.20   # placeholders are lowered until their 3-D clearance to any surface reaches this
RAIL_HAND_OFFSET_M = 0.10      # Handrail_B hand target, aisle side of the grip axis (0.06 m left < 0.05 m clearance)
ROUTE_MAX_RISE_M = 0.80   # placeholders at most this far above the route centre line: the US Lab ceiling anchors (+0.80 m)
                          # are where the traverse captured; higher Node2/Columbus ceilings left 0.8-1.3 m anchor gaps
PLACEHOLDER_PROVENANCE = ("PLACEHOLDER: no public ISS handrail layout; virtual handhold ROUTE_STANDOFF below the "
                          "overhead mesh surface found by ray-casting the source meshes along the route centre line")
PLACEHOLDER_FREE_PROVENANCE = ("PLACEHOLDER (free-standing): no public ISS handrail layout; the overhead surface is more "
                               "than ROUTE_MAX_RISE above the route centre line, so the virtual handhold is held at that "
                               "height with no structure behind it")
PANEL_STATION_BACK_M = 0.35
PANEL_STATION_PROVENANCE = ("PLACEHOLDER (panel station): task-defined virtual handhold PANEL_STATION_BACK aft of the "
                            "maintenance-panel target at the placeholder height; no public ISS handrail layout")
# hand-over-hand scenarios: name -> route polyline (start, hatch centres, end); all 7 modules except the Cupola, whose
# Node3 hatch passage is UNKNOWN and whose nadir descent is the planner base chain's gimbal-lock orientation
ROUTES = ("main", "jem", "node3")


def ray_hits(tris, origins, d):
    """Distance along unit direction d from each origin to the nearest triangle (Moller-Trumbore), inf if none."""
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    h = np.cross(d, e2)
    a = np.einsum("ij,ij->i", e1, h)
    ok = np.abs(a) > 1e-12
    e1, e2, h, a, v0 = e1[ok], e2[ok], h[ok], a[ok], tris[ok, 0]
    out = []
    for o in origins:
        s = o - v0
        u = np.einsum("ij,ij->i", s, h) / a
        q = np.cross(s, e1)
        v = (q @ d) / a
        t = np.einsum("ij,ij->i", e2, q) / a
        m = (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)
        out.append(float(t[m].min()) if m.any() else np.inf)
    return np.asarray(out)


def build_route_anchors(stage, M, hatches, name="main"):
    """Hand-over-hand route anchors on the overhead side of the route centre line (affordance layer only).

    Routes: main = Node1 -> US Lab -> Node2 -> Columbus, jem = Node2 -> JEM PM, node3 = Node1 -> Node3, through the
    measured hatch centres. Anchors every ROUTE_SPACING_M:
    'hatch_rim' between the two measured hatch end planes (+-0.10 m): the measured opening's top edge minus the standoff
    (passage walls are authored cubes, not ray-cast); Handrail_B (the S1 rail)
    replaces the nearest one, all others are 'graspable_placeholder' (no public handrail layout exists).
    """
    hc = {r["pair"]: np.asarray(r["centre_m"]) for r in hatches if r.get("centre_m")}
    hrow = {r["pair"]: r for r in hatches if r.get("centre_m")}
    if name == "main":
        pts = [hc["Node1-USLab"] + np.array([-0.70, 0, 0]), hc["Node1-USLab"], hc["USLab-Node2"],
               np.array([hc["Node2-Columbus"][0], hc["USLab-Node2"][1], hc["USLab-Node2"][2]]),
               hc["Node2-Columbus"], hc["Node2-Columbus"] + np.array([0, -1.0, 0])]
    else:      # from the parent node's centre line through the hatch, along +y to ~1.2 m before the far end
        h = hc["Node2-JEM_PM" if name == "jem" else "Node1-Node3"]
        pts = [np.array([h[0], 0.3, h[2]]), h, h + np.array([0, 7.0 if name == "jem" else 5.2, 0])]   # JEM: y > 9.5
        # m has end-cone hardware 0.08 m above the centre line and open ports
    samples = []
    for a, b in zip(pts[:-1], pts[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / ROUTE_SPACING_M)))
        samples += [a + (b - a) * k / n for k in range(n)]
    samples.append(pts[-1])
    tris = np.concatenate([world_triangles(stage, f"{ISS}/Visual/{m}") for m in MODULES] +
                          [world_triangles(stage, f"{ISS}/Visual/Passages")])
    up = np.array([0.0, 0.0, 1.0])
    dist = ray_hits(tris, np.asarray(samples), up)
    to_station = np.asarray(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), STATION_TO_SCENE_RX_DEG)).GetInverse())
    rail_b = stage.GetPrimAtPath(f"{ISS}/Affordances/USLab_S1Frame/Handrail_B_Graspable")
    rb = np.asarray(UsdGeom.Xformable(rail_b).ComputeLocalToWorldTransform(0).ExtractTranslation())
    rb_hand = rb - up * RAIL_HAND_OFFSET_M
    i_rb = int(np.argmin([np.linalg.norm(s[:2] - rb[:2]) for s in samples])) if name == "main" else -1
    # panel station (main only): the sample nearest PANEL_STATION_BACK_M aft of the maintenance-panel target becomes
    # a virtual handhold from which the panel is within reach (from Handrail_B it is 0.93-0.96 m, G1's span limit)
    i_ps, ps = -1, None
    if name == "main":
        tgt = np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(f"{ISS}/Affordances/USLab_S1Frame/MaintenancePanel_Target"))
                         .ComputeLocalToWorldTransform(0).ExtractTranslation())
        ps = np.array([tgt[0] - PANEL_STATION_BACK_M, 0.0, 0.0])
        i_ps = int(np.argmin([abs(s[0] - ps[0]) if k != i_rb else 1e9 for k, s in enumerate(samples)]))
    root = UsdGeom.Xform.Define(stage, f"{ISS}/Affordances/Route" + ("" if name == "main" else f"_{name}")).GetPrim()
    import trimesh
    cube = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
    ctri = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
                     [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    boxes = []          # every collision box (US Lab proxies, passage walls) as triangles, scene frame
    for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{ISS}/Collision")):
        if prim.IsA(UsdGeom.Cube):
            m = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
            v = (np.c_[cube * (UsdGeom.Cube(prim).GetSizeAttr().Get() or 2.0), np.ones(8)] @ m)[:, :3]
            boxes.append(v[ctri])
    allt = np.concatenate([tris] + [world_triangles(stage, f"{ISS}/Collision/{m}") for m in MODULES if m != "USLab"]
                          + boxes)
    prox = trimesh.proximity.ProximityQuery(trimesh.Trimesh(vertices=allt.reshape(-1, 3),
                                                            faces=np.arange(len(allt) * 3).reshape(-1, 3), process=False))

    def clearance(x):
        return float(prox.on_surface(np.asarray([x]))[1][0])

    out, skipped = [], []
    for i, (s, dz) in enumerate(zip(samples, dist)):
        def in_span(k):
            ax = "xyz".index(hrow[k]["axis"])
            lo, hi = sorted((hrow[k]["a"]["end_m"], hrow[k]["b"]["end_m"]))
            return lo - 0.10 <= s[ax] <= hi + 0.10 and np.linalg.norm((s - hc[k])[[0, 1]]) < 0.8
        near = [k for k in hc if in_span(k)]       # between the two measured hatch end planes (+-0.10 m)
        kind, prov, vis = "graspable_placeholder", PLACEHOLDER_PROVENANCE, None
        if i == i_rb:
            p, kind, prov = rb_hand, "graspable_rail", "S1 Handrail_B (NASA handrail mesh, S1 placement)"
            vis = f"{ISS}/Visual/USLab_S1Frame/Handrail_B"
        elif i == i_ps:
            p = np.array([ps[0], s[1], s[2] + ROUTE_MAX_RISE_M])
            while clearance(p) < ROUTE_MIN_CLEARANCE_M and p[2] > s[2]:
                p = p - up * 0.01
            prov = PANEL_STATION_PROVENANCE
        elif near:
            r = hrow[near[0]]
            top = min(r["a"]["size_m"][1], r["b"]["size_m"][1]) / 2
            p = np.array([s[0], s[1], hc[near[0]][2] + top - ROUTE_STANDOFF_M])
            kind, prov = "hatch_rim", f"measured hatch {near[0]}: top edge of the smaller measured opening"
        elif np.isfinite(dz) or name != "main":     # main keeps its skip (Node1 open port, S1.4 receipts)
            p = s + up * min(dz - ROUTE_STANDOFF_M, ROUTE_MAX_RISE_M)   # no surface (open port): free-standing
            if dz - ROUTE_STANDOFF_M > ROUTE_MAX_RISE_M + 0.05:     # >5 cm above the cap: no surface behind it
                prov = PLACEHOLDER_FREE_PROVENANCE
            while clearance(p) < ROUTE_MIN_CLEARANCE_M and p[2] > s[2]:   # lower toward the module axis
                p = p - up * 0.01
            if clearance(p) < ROUTE_MIN_CLEARANCE_M and name != "main":
                # lowering reached the axis without clearance (equipment over the aisle, Node3): try the same
                # placement shifted sideways, keep the highest candidate with clearance (main keeps its receipts)
                side = np.cross(up, (pts[-1] - pts[0]) / np.linalg.norm(pts[-1] - pts[0]))
                best = None
                for off in (0.15, -0.15, 0.30, -0.30, 0.45, -0.45):
                    s2 = s + side * off
                    dz2 = float(ray_hits(tris, np.asarray([s2]), up)[0])
                    q = s2 + up * min(dz2 - ROUTE_STANDOFF_M, ROUTE_MAX_RISE_M)
                    while clearance(q) < ROUTE_MIN_CLEARANCE_M and q[2] > s2[2]:
                        q = q - up * 0.01
                    if clearance(q) >= ROUTE_MIN_CLEARANCE_M and (best is None or q[2] > best[2]):
                        best = q
                if best is not None:
                    p = best
                    prov = prov + "; shifted sideways from the route centre line (no clearance above it)"
        else:
            skipped.append(np.round(s, 3).tolist())          # ray leaves through an open port: no surface
            continue
        module = min(MODULES, key=lambda m: np.linalg.norm(
            np.asarray(UsdGeom.BBoxCache(0, ["default", "render"]).ComputeWorldBound(
                stage.GetPrimAtPath(f"{ISS}/Visual/{m}")).ComputeAlignedRange().GetMidpoint()) - s))
        g = S1.box(stage, f"{root.GetPath()}/Anchor_{len(out):02d}", tuple((np.r_[p, 1.0] @ to_station)[:3]),
                   (0.04, 0.04, 0.04), M["grasp"] if kind != "graspable_placeholder" else M["waypoint"]).GetPrim()
        g.CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set(kind)
        g.CreateAttribute("iss:routeIndex", Sdf.ValueTypeNames.Int).Set(len(out))
        g.CreateRelationship("iss:visualSource").SetTargets([Sdf.Path(vis or f"{ISS}/Visual/{module}")])
        g.CreateRelationship("iss:collisionProxy").SetTargets([Sdf.Path(f"{ISS}/Collision/{module}")])
        g.SetCustomDataByKey("provenance", prov)
        out.append({"index": len(out), "kind": kind, "pos_scene_m": np.round(p, 4).tolist(),
                    "clearance_m": round(clearance(p), 4),
                    "surface_dist_m": round(float(dz), 4) if np.isfinite(dz) else None, "module": module,
                    "free_standing": prov == PLACEHOLDER_FREE_PROVENANCE, "panel_station": i == i_ps})
    gaps = [float(np.linalg.norm(np.asarray(a["pos_scene_m"]) - np.asarray(b["pos_scene_m"]))) for a, b in zip(out, out[1:])]
    u = (pts[1] - pts[0]) / np.linalg.norm(pts[1] - pts[0])
    ctr = np.mean([a["pos_scene_m"] for a in out], axis=0)
    return {"anchors": out, "skipped_samples": skipped, "max_gap_m": round(max(gaps), 4) if gaps else None,
            "start_pelvis_m": np.round([out[0]["pos_scene_m"][0] - 0.45 * u[0], out[0]["pos_scene_m"][1] - 0.45 * u[1],
                                        pts[0][2]], 4).tolist(), "facing": u.round(4).tolist(),
            "frame_offset_m": [3.0, 0.0, -4.84] if name == "main" else np.round([ctr[0], ctr[1], -4.84], 2).tolist()}


LAYERS = {
    #             Visual  Collision  Affordances  Task
    "visual":     (True,  False,     False,       False),
    "collision":  (False, True,      False,       False),
    "affordance": (False, True,      True,        True),
}
LAYER_PRIMS = [f"{ISS}/Visual", f"{ISS}/Collision", f"{ISS}/Affordances", "/World/Task"]


def build_variants(stage, M):
    vset = stage.GetPrimAtPath("/World").GetVariantSets().AddVariantSet("truthLayer")
    col = [p.GetPath() for p in Usd.PrimRange(stage.GetPrimAtPath(LAYER_PRIMS[1])) if p.IsA(UsdGeom.Gprim)]
    for name, flags in LAYERS.items():
        vset.AddVariant(name)
        vset.SetVariantSelection(name)
        with vset.GetVariantEditContext():
            for path, on in zip(LAYER_PRIMS, flags):
                UsdGeom.Imageable(stage.GetPrimAtPath(path)).CreateVisibilityAttr().Set(
                    UsdGeom.Tokens.inherited if on else UsdGeom.Tokens.invisible)
            for p in col:
                task = "/Handrail_" in str(p) or str(p).endswith("/MaintenancePanel")
                mat = M["context"] if name == "affordance" else (M["collision"] if task else M["collision_env"])
                UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(p)).Bind(mat, UsdShade.Tokens.strongerThanDescendants)
    vset.SetVariantSelection("visual")


def main():
    needed = [MEDIA / f"{m}.usd" for m, _ in MODULES.values()] + [MEDIA / "handrail_30.usd"]
    missing = [str(f) for f in needed if not f.exists()]
    if missing:
        raise SystemExit(f"Missing {missing}; run scripts/import_iss_media.py first")
    poses = urdf_poses()
    SCENE.parent.mkdir(parents=True, exist_ok=True)
    if SCENE.exists():
        SCENE.unlink()
    stage = Usd.Stage.CreateNew(str(SCENE))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetMetadata("comment", "Full ISS interior v0 (USOS modules in nasa/astrobee_media). Visual: NASA meshes posed by "
                                 "astrobee_iss/urdf/model.urdf. Collision: US Lab S1 proxies + triangle meshes elsewhere. "
                                 "Not a digital twin. Built by scripts/build_iss_full.py")
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    for p in ("/World/Environment", "/World/Actors", "/World/Task"):
        UsdGeom.Xform.Define(stage, p)
    iss = UsdGeom.Xform.Define(stage, ISS)
    iss.AddRotateXOp(opSuffix="stationToScene").Set(STATION_TO_SCENE_RX_DEG)
    ip = iss.GetPrim()
    ip.CreateAttribute("iss:frame", Sdf.ValueTypeNames.String).Set(
        "children are in the Astrobee ISS world frame (x fwd, y starboard, z nadir); this prim rotates it 180 deg "
        "about x so scene +Z = zenith")
    ip.CreateAttribute("iss:knownGaps", Sdf.ValueTypeNames.StringArray).Set(KNOWN_GAPS)
    ip.CreateAttribute("iss:sourceUrdf", Sdf.ValueTypeNames.String).Set(
        "nasa/astrobee_media@1cb0620 astrobee_iss/urdf/model.urdf sha256 " + sha256(URDF))
    ip.CreateAttribute("iss:license", Sdf.ValueTypeNames.String).Set(
        "astrobee_iss/package.xml: Apache-2.0; astrobee_media readme: all multimedia covered by NASA Media Usage "
        "Guidelines; interior meshes from NASA 3D Resources ISS Internal (IGOAL). Governing terms UNKNOWN.")
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(0.0)   # IVA microgravity; no orbital mechanics
    M = S1.build_materials(stage)
    vis = build_visual(stage, poses)
    hatches = measure_hatches(stage, f"{ISS}/Visual")
    du = np.asarray(poses["us_lab"]["collision"]["xyz"]) - np.asarray(poses["us_lab"]["visual"]["xyz"])
    shift_scene = np.asarray(to_station_point(du)).tolist()   # the 180 deg x-rotation is its own inverse
    hatches_col = measure_hatches(stage, f"{ISS}/Visual", shift={"USLab": shift_scene})
    strategy = build_collision(stage, poses, vis["USLab"])
    build_affordances(stage, M, vis["USLab"], hatches)
    passages = build_passages(stage, M, hatches)
    routes = {n: build_route_anchors(stage, M, hatches, n) for n in ROUTES}
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    boxes = {}
    for name in MODULES:
        r = bbc.ComputeWorldBound(stage.GetPrimAtPath(f"{ISS}/Visual/{name}")).ComputeAlignedRange()
        boxes[name] = (np.asarray(r.GetMin()), np.asarray(r.GetMax()))
    build_lighting(stage, vis["USLab"], boxes)
    cams = build_cameras(stage, boxes, hatches)
    build_variants(stage, M)
    stage.GetRootLayer().Save()
    text = SCENE.read_text(encoding="utf-8")
    SCENE.write_text(text.rstrip("\n") + "\n", encoding="utf-8", newline="\n")
    report = {"scene": SCENE.relative_to(ROOT).as_posix(), "scene_sha256": sha256(SCENE), "urdf_sha256": sha256(URDF),
              "poses": poses, "hatch_tolerance": HATCH_TOL, "hatches_visual_origin": hatches,
              "hatches_uslab_collision_origin": hatches_col, "collision_strategy": strategy, "cameras": cams,
              "module_bbox_scene_m": {k: [v[0].round(3).tolist(), v[1].round(3).tolist()] for k, v in boxes.items()},
              "known_gaps": KNOWN_GAPS, "authored_passages": passages, "passage_provenance": PASSAGE_PROVENANCE,
              "route_anchors": routes["main"], "routes": routes, "placeholder_provenance": PLACEHOLDER_PROVENANCE}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "build_report.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    for r in hatches:
        print(r["pair"], r.get("centre_offset_m"), r.get("axial_gap_m"), r["within_tolerance"],
              (r["a"] or {}).get("status"), (r["b"] or {}).get("status"))
    print("US Lab at URDF collision origin:", [(r["pair"], r.get("centre_offset_m"), r.get("axial_gap_m"))
                                             for r in hatches_col if "USLab" in r["pair"]])
    print(json.dumps({"scene": str(SCENE), "cameras": list(cams), "collision": {k: v.split(":")[0] for k, v in strategy.items()}}))


if __name__ == "__main__":
    main()

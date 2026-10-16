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
              "known_gaps": KNOWN_GAPS}
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

"""Build the S1 ISS Micro-Lab v0 scene (Visual / Collision / Affordance truth layers).

Pure pxr authoring; run with C:\\isaacsim\\python.bat scripts/build_iss_microlab.py
The environment is ISS-INSPIRED authored geometry (not a digital twin). R2 is referenced from the
scripted zero-g derivative import and posed KINEMATICALLY (joint xform overrides, physics disabled).
"""
import hashlib
import json
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_microlab_v0.usda"
R2_USD = ROOT / "outputs" / "r2_derived" / "isaac_import" / "r2c6_zero_g" / "r2c6_zero_g.usda"
R2_URDF = ROOT / "outputs" / "r2_derived" / "r2c6_zero_g.urdf"

# ---- authored dimensions (metres) -------------------------------------------------------------
HALF = 1.10            # aisle half-width/height: rack faces at |y|,|z| = 1.10 (2.2 m clear aisle)
CHAMFER = 0.35         # 45-degree corner standoff width (projected on each axis)
BAY_X = [-1.575, -0.525, 0.525, 1.575]  # four 1.05 m rack bays along the module axis (+X)
X_MIN, X_MAX = -2.15, 2.15
HANDRAIL = {"grip_len": 0.45, "grip_w": 0.033, "grip_t": 0.024, "standoff": 0.065, "standoff_w": 0.030}

# Kinematic R2 pose (degrees, all within URDF limits). Visual storytelling only, never simulated.
# Fixed joints set directly; limb joints solved by a small coordinate-descent IK toward authored targets
# (left palm -> Handrail_B, right palm relaxed, feet in a bent-knee neutral-body-posture envelope).
R2_ROOT = {"pos": (0.60, 0.05, 0.06), "rpy_deg": (0.0, 14.0, -148.0)}
R2_POSE = {"waistjoint0": 12.0, "neckjoint0": -10.0, "left_armjoint2": -60.0, "right_armjoint2": 60.0}
R2_IK = {   # chain joints, end-effector link, target ("world" point or "body" point in R2 root frame)
    "left_arm": (["left_armjoint0", "left_armjoint1", "left_armjoint3", "left_armwristpitch"], "left_palm", "world", "handrail_B"),
    "right_arm": (["right_armjoint0", "right_armjoint1", "right_armjoint3", "right_armwristpitch"], "right_palm", "body", (0.30, -0.50, 0.12)),
    "left_leg": (["left_legjoint0", "left_legjoint1", "left_legjoint3", "left_legjoint5"], "left_leg_foot", "body", (-0.20, 0.17, -0.80)),
    "right_leg": (["right_legjoint0", "right_legjoint1", "right_legjoint3", "right_legjoint5"], "right_leg_foot", "body", (-0.34, -0.13, -0.64)),
}


# ---- authoring helpers ------------------------------------------------------------------------
def material(stage, path, color, rough=0.6, metal=0.0, emissive=None, opacity=1.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    if emissive:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    if opacity < 1.0:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _place(prim, center, scale=(1, 1, 1), rot_xyz_deg=(0, 0, 0)):
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*center))
    if any(rot_xyz_deg):
        x.AddRotateXYZOp().Set(Gf.Vec3f(*rot_xyz_deg))
    x.AddScaleOp().Set(Gf.Vec3f(*scale))


def box(stage, path, center, size, mat=None, rot=(0, 0, 0)):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    _place(c.GetPrim(), center, size, rot)
    if mat:
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
    return c


def cyl(stage, path, center, radius, height, axis="X", mat=None, rot=(0, 0, 0)):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.CreateAxisAttr(axis)
    _place(c.GetPrim(), center, (1, 1, 1), rot)
    if mat:
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
    return c


def disc(stage, path, center, radius, mat, segments=64):
    """Flat +Z-facing circular mesh (renderer-independent tessellation, unlike UsdGeom.Cylinder)."""
    m = UsdGeom.Mesh.Define(stage, path)
    pts = [Gf.Vec3f(0, 0, 0)] + [Gf.Vec3f(radius * math.cos(2 * math.pi * i / segments),
                                          radius * math.sin(2 * math.pi * i / segments), 0) for i in range(segments)]
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr([3] * segments)
    m.CreateFaceVertexIndicesAttr([v for i in range(segments) for v in (0, 1 + i, 1 + (i + 1) % segments)])
    m.CreateExtentAttr([Gf.Vec3f(-radius, -radius, 0), Gf.Vec3f(radius, radius, 0)])
    m.CreateSubdivisionSchemeAttr("none")
    _place(m.GetPrim(), center)
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def wall_frame(wall):
    """Return (in-wall axis index, normal sign, normal axis index) for the four rack walls."""
    return {"port": (2, +1, 1), "starboard": (2, -1, 1), "overhead": (1, +1, 2), "deck": (1, -1, 2)}[wall]


def wall_point(wall, x, u, depth):
    """Point on a wall: x along module, u along the wall's in-plane axis, depth = offset from face toward wall."""
    in_ax, sign, n_ax = wall_frame(wall)
    p = [x, 0.0, 0.0]
    p[n_ax] = sign * (HALF + depth)
    p[in_ax] = u
    return tuple(p)


def wall_size(wall, sx, su, sn):
    in_ax, _, n_ax = wall_frame(wall)
    s = [sx, 0.0, 0.0]
    s[in_ax] = su
    s[n_ax] = sn
    return tuple(s)


# ---- Visual layer -----------------------------------------------------------------------------
def build_materials(stage):
    m = "/World/Looks"
    UsdGeom.Scope.Define(stage, m)
    return {
        "rack": material(stage, f"{m}/RackWhite", (0.80, 0.80, 0.77), 0.55),
        "rack_back": material(stage, f"{m}/RackBack", (0.33, 0.34, 0.35), 0.7),
        "fabric": material(stage, f"{m}/Nomex", (0.76, 0.73, 0.66), 0.95),
        "standoff": material(stage, f"{m}/Standoff", (0.68, 0.69, 0.70), 0.5),
        "metal": material(stage, f"{m}/Aluminium", (0.74, 0.75, 0.77), 0.32, 1.0),
        "handrail": material(stage, f"{m}/HandrailAnodized", (0.52, 0.60, 0.69), 0.35, 0.6),
        "dark": material(stage, f"{m}/DarkPolymer", (0.07, 0.07, 0.08), 0.6),
        "cable": material(stage, f"{m}/CableJacket", (0.26, 0.27, 0.29), 0.75),
        "task_panel": material(stage, f"{m}/TaskPanel", (0.60, 0.62, 0.64), 0.45, 0.2),
        "luminaire": material(stage, f"{m}/LuminaireDiffuser", (0.9, 0.9, 0.88), 0.4, emissive=(4.0, 3.9, 3.6)),
        "window": material(stage, f"{m}/WindowEarthGlow", (0.02, 0.05, 0.10), 0.05, emissive=(0.10, 0.32, 0.95)),
        "r2_shell": material(stage, f"{m}/R2ShellPlain", (0.86, 0.86, 0.84), 0.45),
        "collision_env": material(stage, f"{m}/CollisionProxyBounds", (0.60, 0.61, 0.62), 0.85),
        "collision": material(stage, f"{m}/CollisionProxyTask", (0.90, 0.56, 0.20), 0.7),
        "context": material(stage, f"{m}/CollisionContext", (0.30, 0.31, 0.32), 0.9),
        "grasp": material(stage, f"{m}/AffordanceGraspable", (0.15, 0.65, 0.55), 0.5),
        "target": material(stage, f"{m}/AffordanceMaintenance", (0.30, 0.55, 0.95), 0.5),
        "waypoint": material(stage, f"{m}/AffordanceWaypoint", (0.92, 0.92, 0.92), 0.5),
    }


def rack(stage, root, wall, bi, x, M):
    p = f"{root}/{wall.capitalize()}_Rack{bi}"
    UsdGeom.Xform.Define(stage, p)
    box(stage, f"{p}/Back", wall_point(wall, x, 0, 0.03), wall_size(wall, 1.02, 1.50, 0.04), M["rack_back"])
    kind = (bi * 3 + len(wall)) % 3
    if kind == 0:   # stowage lockers: 2 x 4 grid with latches
        for i, dx in enumerate((-0.25, 0.25)):
            for j, u in enumerate((-0.54, -0.18, 0.18, 0.54)):
                q = f"{p}/Locker_{i}{j}"
                box(stage, q, wall_point(wall, x + dx, u, 0.004), wall_size(wall, 0.484, 0.344, 0.02), M["rack"])
                box(stage, f"{q}_Latch", wall_point(wall, x + dx, u + 0.13, -0.012),
                    wall_size(wall, 0.06, 0.03, 0.012), M["metal"])
    elif kind == 1:  # soft-goods (fabric) closeout with two straps
        box(stage, f"{p}/Fabric", wall_point(wall, x, 0, 0.0), wall_size(wall, 0.98, 1.46, 0.03), M["fabric"])
        for k, u in enumerate((-0.4, 0.4)):
            box(stage, f"{p}/Strap{k}", wall_point(wall, x, u, -0.017), wall_size(wall, 0.98, 0.035, 0.006), M["standoff"])
    else:            # payload rack: front panel + recessed experiment drawer + connector strip
        box(stage, f"{p}/Front", wall_point(wall, x, 0.25, 0.004), wall_size(wall, 0.98, 0.98, 0.02), M["rack"])
        box(stage, f"{p}/Drawer", wall_point(wall, x, -0.50, 0.02), wall_size(wall, 0.90, 0.44, 0.02), M["standoff"])
        box(stage, f"{p}/DrawerHandle", wall_point(wall, x, -0.50, -0.01), wall_size(wall, 0.30, 0.025, 0.02), M["metal"])
        for k in range(5):
            box(stage, f"{p}/Connector{k}", wall_point(wall, x - 0.32 + 0.16 * k, 0.62, -0.012),
                wall_size(wall, 0.05, 0.05, 0.012), M["dark"])


def corner_geom(sy, sz, inset=0.0, thickness=0.03):
    """Center and X-rotation of a 45-degree corner standoff plate (sy, sz = +/-1)."""
    th = math.degrees(math.atan2(-sy, sz))
    d = HALF - CHAMFER / 2
    off = thickness / 2 + inset   # push outward (toward the hull) by half thickness
    c = (sy * (d + off / math.sqrt(2)), sz * (d + off / math.sqrt(2)))
    return c, th


def handrail_dims():
    h = HANDRAIL
    return h["grip_len"], h["grip_w"], h["grip_t"], h["standoff"], h["standoff_w"]


def handrail(stage, root, name, wall, x, u, mat, collision=False):
    """Straight IVA-style handrail along X: grip bar on two standoffs, mounted on a rack face."""
    L, gw, gt, so, sw = handrail_dims()
    p = f"{root}/{name}"
    UsdGeom.Xform.Define(stage, p)
    grip_depth = -(so + gt / 2)
    grip = box(stage, f"{p}/Grip", wall_point(wall, x, u, grip_depth), wall_size(wall, L, gw, gt), mat)
    for k, dx in enumerate((-L / 2 + sw / 2, L / 2 - sw / 2)):
        box(stage, f"{p}/Standoff{k}", wall_point(wall, x + dx, u, -so / 2), wall_size(wall, sw, gw, so), mat)
    if collision:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(p)):
            if prim.IsA(UsdGeom.Gprim):
                UsdPhysics.CollisionAPI.Apply(prim)
    return grip


def build_visual(stage, M, layout):
    root = "/World/Environment/ISS/Visual"
    UsdGeom.Xform.Define(stage, root)
    for wall in ("port", "starboard", "overhead", "deck"):
        for bi, x in enumerate(BAY_X):
            if wall == "deck" and bi == 2:
                continue  # nadir window bay
            rack(stage, root, wall, bi, x, M)
    # nadir window (Earth-glow cue): surround, frame and glazing
    wx = BAY_X[2]
    w = f"{root}/NadirWindow"
    UsdGeom.Xform.Define(stage, w)
    box(stage, f"{w}/Surround", wall_point("deck", wx, 0, 0.03), wall_size("deck", 1.02, 1.50, 0.04), M["standoff"])
    for k, (dx, du, sx, su) in enumerate(((0, 0.33, 0.70, 0.06), (0, -0.33, 0.70, 0.06), (0.33, 0, 0.06, 0.60), (-0.33, 0, 0.06, 0.60))):
        box(stage, f"{w}/Frame{k}", wall_point("deck", wx + dx, du, -0.01), wall_size("deck", sx, su, 0.04), M["metal"])
    disc(stage, f"{w}/Glazing", wall_point("deck", wx, 0, 0.007), 0.25, M["window"])
    # corner standoffs: luminaires on the two overhead corners, cable runs on the two deck corners
    for sy in (+1, -1):
        for sz in (+1, -1):
            tag = f"{'P' if sy > 0 else 'S'}{'O' if sz > 0 else 'D'}"
            (cy, cz), th = corner_geom(sy, sz)
            plen = X_MAX - X_MIN
            box(stage, f"{root}/Standoff_{tag}", (0.0, cy, cz), (plen, CHAMFER * math.sqrt(2), 0.03), M["standoff"], (th, 0, 0))
            if sz > 0:
                (ly, lz), _ = corner_geom(sy, sz, inset=-0.018, thickness=0.012)
                for bi, x in enumerate(BAY_X):
                    box(stage, f"{root}/Luminaire_{tag}{bi}", (x, ly, lz), (0.85, 0.10, 0.012), M["luminaire"], (th, 0, 0))
            else:
                d = HALF - CHAMFER / 2 - 0.04
                for k, (r, o) in enumerate(((0.018, -0.05), (0.013, 0.0), (0.022, 0.055))):
                    ay = sy * (d + o * 0.707) ; az = sz * (d - o * 0.707)
                    cyl(stage, f"{root}/Cable_{tag}{k}", (0.0, ay, az), r, plen, "X", M["cable"])
                for k, x in enumerate((-1.6, -0.5, 0.6, 1.7)):
                    box(stage, f"{root}/CableClamp_{tag}{k}", (x, sy * d, sz * d), (0.03, 0.09, 0.09), M["dark"], (th, 0, 0))
    # bulkheads: aft closed, forward with square hatchway into a dim adjacent volume
    box(stage, f"{root}/AftBulkhead", (X_MIN - 0.03, 0, 0), (0.06, 2 * HALF + 0.1, 2 * HALF + 0.1), M["standoff"])
    hw = 0.60
    fwd = f"{root}/ForwardHatchway"
    UsdGeom.Xform.Define(stage, fwd)
    for k, (cy, cz, sy_, sz_) in enumerate(((0, (HALF + hw) / 2, 2 * HALF, HALF - hw), (0, -(HALF + hw) / 2, 2 * HALF, HALF - hw),
                                            ((HALF + hw) / 2, 0, HALF - hw, 2 * hw), (-(HALF + hw) / 2, 0, HALF - hw, 2 * hw))):
        box(stage, f"{fwd}/Bulkhead{k}", (X_MAX + 0.03, cy, cz), (0.06, sy_ + 0.1, sz_ + 0.1), M["standoff"])
    for k, (cy, cz, sy_, sz_) in enumerate(((0, hw + 0.03, 2 * hw + 0.12, 0.06), (0, -hw - 0.03, 2 * hw + 0.12, 0.06),
                                            (hw + 0.03, 0, 0.06, 2 * hw), (-hw - 0.03, 0, 0.06, 2 * hw))):
        box(stage, f"{fwd}/HatchRim{k}", (X_MAX - 0.01, cy, cz), (0.08, sy_, sz_), M["metal"])
    for k, (cy, cz, sy_, sz_) in enumerate(((0, hw, 2 * hw, 0.02), (0, -hw, 2 * hw, 0.02), (hw, 0, 0.02, 2 * hw), (-hw, 0, 0.02, 2 * hw))):
        box(stage, f"{fwd}/Vestibule{k}", (X_MAX + 0.40, cy, cz), (0.70, sy_, sz_), M["rack_back"])
    box(stage, f"{fwd}/AdjacentModuleWall", (X_MAX + 2.2, 0, 0), (0.05, 2 * HALF, 2 * HALF), M["rack"])
    for k, (cy, cz, sy_, sz_) in enumerate(((0, HALF, 2 * HALF, 0.02), (0, -HALF, 2 * HALF, 0.02), (HALF, 0, 0.02, 2 * HALF), (-HALF, 0, 0.02, 2 * HALF))):
        box(stage, f"{fwd}/AdjacentModule{k}", (X_MAX + 1.45, cy, cz), (1.5, sy_, sz_), M["rack"])
    # handrails (visual) and maintenance panel (visual)
    for name, spec in layout["handrails"].items():
        handrail(stage, root, name, spec["wall"], spec["x"], spec["u"], M["handrail"])
    tp = layout["panel"]
    pp = f"{root}/MaintenancePanel"
    UsdGeom.Xform.Define(stage, pp)
    box(stage, f"{pp}/Plate", wall_point(tp["wall"], tp["x"], tp["u"], -0.012), wall_size(tp["wall"], tp["sx"], tp["su"], 0.024), M["task_panel"])
    for k, (dx, du) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        cyl(stage, f"{pp}/Fastener{k}", wall_point(tp["wall"], tp["x"] + dx * (tp["sx"] / 2 - 0.03), tp["u"] + du * (tp["su"] / 2 - 0.03), -0.027),
            0.009, 0.008, "Y" if tp["wall"] in ("port", "starboard") else "Z", M["metal"])
    box(stage, f"{pp}/Connector", wall_point(tp["wall"], tp["x"] + 0.06, tp["u"], -0.035), wall_size(tp["wall"], 0.10, 0.06, 0.022), M["dark"])
    box(stage, f"{pp}/Label", wall_point(tp["wall"], tp["x"] - 0.08, tp["u"] + 0.07, -0.0245), wall_size(tp["wall"], 0.12, 0.035, 0.001), M["rack"])
    return root


# ---- Collision / Affordance / Task ------------------------------------------------------------
def build_collision(stage, M, layout):
    root = "/World/Environment/ISS/Collision"
    UsdGeom.Xform.Define(stage, root)
    L = X_MAX - X_MIN
    slabs = {
        "PortWall": ((0, HALF + 0.025, 0), (L, 0.05, 2 * HALF)),
        "StarboardWall": ((0, -HALF - 0.025, 0), (L, 0.05, 2 * HALF)),
        "OverheadWall": ((0, 0, HALF + 0.025), (L, 2 * HALF, 0.05)),
        "DeckWall": ((0, 0, -HALF - 0.025), (L, 2 * HALF, 0.05)),
        "AftBulkhead": ((X_MIN - 0.025, 0, 0), (0.05, 2 * HALF, 2 * HALF)),
    }
    hw = 0.60
    for k, (cy, cz, sy_, sz_) in enumerate(((0, (HALF + hw) / 2, 2 * HALF, HALF - hw), (0, -(HALF + hw) / 2, 2 * HALF, HALF - hw),
                                            ((HALF + hw) / 2, 0, HALF - hw, 2 * hw), (-(HALF + hw) / 2, 0, HALF - hw, 2 * hw))):
        slabs[f"ForwardBulkhead{k}"] = ((X_MAX + 0.025, cy, cz), (0.05, sy_, sz_))
    for k, (cy, cz, sy_, sz_) in enumerate(((0, hw + 0.01, 2 * hw, 0.02), (0, -hw - 0.01, 2 * hw, 0.02),
                                            (hw + 0.01, 0, 0.02, 2 * hw), (-hw - 0.01, 0, 0.02, 2 * hw))):
        slabs[f"Vestibule{k}"] = ((X_MAX + 0.40, cy, cz), (0.70, sy_, sz_))   # hatchway tunnel: still a real boundary
    slabs["AdjacentModuleWall"] = ((X_MAX + 2.2, 0, 0), (0.05, 2 * HALF, 2 * HALF))
    for k, (cy, cz, sy_, sz_) in enumerate(((0, HALF, 2 * HALF, 0.02), (0, -HALF, 2 * HALF, 0.02),
                                            (HALF, 0, 0.02, 2 * HALF), (-HALF, 0, 0.02, 2 * HALF))):
        slabs[f"AdjacentModule{k}"] = ((X_MAX + 1.45, cy, cz), (1.5, sy_, sz_))
    for name, (c, s) in slabs.items():
        UsdPhysics.CollisionAPI.Apply(box(stage, f"{root}/{name}", c, s).GetPrim())
    for sy in (+1, -1):
        for sz in (+1, -1):
            (cy, cz), th = corner_geom(sy, sz, thickness=0.05)
            tag = f"{'P' if sy > 0 else 'S'}{'O' if sz > 0 else 'D'}"
            UsdPhysics.CollisionAPI.Apply(box(stage, f"{root}/Standoff_{tag}", (0, cy, cz), (L, CHAMFER * math.sqrt(2), 0.05), None, (th, 0, 0)).GetPrim())
    for name, spec in layout["handrails"].items():
        handrail(stage, root, name, spec["wall"], spec["x"], spec["u"], None, collision=True)
    tp = layout["panel"]
    UsdPhysics.CollisionAPI.Apply(box(stage, f"{root}/MaintenancePanel", wall_point(tp["wall"], tp["x"], tp["u"], -0.012),
                                      wall_size(tp["wall"], tp["sx"], tp["su"], 0.024)).GetPrim())
    return root


def _set_rel(prim, name, target):
    prim.CreateRelationship(name).SetTargets([Sdf.Path(target)])


def build_affordances(stage, M, layout):
    root = "/World/Environment/ISS/Affordances"
    UsdGeom.Xform.Define(stage, root)
    L, gw, gt, so, _ = handrail_dims()
    pad = 0.004
    for name, spec in layout["handrails"].items():
        p = f"{root}/{name}_Graspable"
        g = box(stage, p, wall_point(spec["wall"], spec["x"], spec["u"], -(so + gt / 2)),
                wall_size(spec["wall"], L - 0.07 + pad, gw + pad, gt + pad), M["grasp"]).GetPrim()
        g.CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set("graspable")
        g.CreateAttribute("iss:graspLength_m", Sdf.ValueTypeNames.Float).Set(L - 0.07)
        g.CreateAttribute("iss:gripSection_m", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(gw, gt))
        _set_rel(g, "iss:collisionProxy", f"/World/Environment/ISS/Collision/{name}/Grip")
        _set_rel(g, "iss:visualSource", f"/World/Environment/ISS/Visual/{name}/Grip")
    tp = layout["panel"]
    t = box(stage, f"{root}/MaintenancePanel_Target", wall_point(tp["wall"], tp["x"], tp["u"], -0.026),
            wall_size(tp["wall"], tp["sx"] + 0.01, tp["su"] + 0.01, 0.004), M["target"]).GetPrim()
    t.CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set("maintenance_target")
    _set_rel(t, "iss:collisionProxy", "/World/Environment/ISS/Collision/MaintenancePanel")
    _set_rel(t, "iss:visualSource", "/World/Environment/ISS/Visual/MaintenancePanel")
    # Task namespace: references into affordances (no dynamics, no controller yet)
    UsdGeom.Xform.Define(stage, "/World/Task/Targets")
    tgt = UsdGeom.Xform.Define(stage, "/World/Task/Targets/MaintenancePanel").GetPrim()
    _place(tgt, wall_point(tp["wall"], tp["x"], tp["u"], -0.03))
    _set_rel(tgt, "iss:affordance", f"{root}/MaintenancePanel_Target")
    UsdGeom.Xform.Define(stage, "/World/Task/Waypoints")
    wp = UsdGeom.Sphere.Define(stage, "/World/Task/Waypoints/R2_Park")
    wp.CreateRadiusAttr(0.035)
    _place(wp.GetPrim(), layout["park"])
    UsdShade.MaterialBindingAPI.Apply(wp.GetPrim()).Bind(M["waypoint"])
    wp.GetPrim().CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set("approach_park")
    _set_rel(wp.GetPrim(), "iss:graspCandidates", f"{root}/Handrail_B_Graspable")
    return root


# ---- Lighting / camera ------------------------------------------------------------------------
LIGHT = {"luminaire_intensity": 28000.0, "luminaire_kelvin": 4800.0, "window_intensity": 9000.0,
         "adjacent_intensity": 2500.0, "fill_intensity": 15.0}


def build_lighting(stage):
    root = "/World/Environment/ISS/Lighting"
    UsdGeom.Xform.Define(stage, root)
    for sy in (+1, -1):
        (ly, lz), th = corner_geom(sy, +1, inset=-0.026, thickness=0.0)
        for bi, x in enumerate(BAY_X):
            lt = UsdLux.RectLight.Define(stage, f"{root}/GLA_{'P' if sy > 0 else 'S'}{bi}")
            lt.CreateWidthAttr(0.85)
            lt.CreateHeightAttr(0.10)
            lt.CreateIntensityAttr(LIGHT["luminaire_intensity"])
            lt.CreateEnableColorTemperatureAttr(True)
            lt.CreateColorTemperatureAttr(LIGHT["luminaire_kelvin"])
            _place(lt.GetPrim(), (x, ly, lz), (1, 1, 1), (th, 0, 0))
    wl = UsdLux.DiskLight.Define(stage, f"{root}/NadirWindow_EarthLight")
    wl.CreateRadiusAttr(0.25)
    wl.CreateIntensityAttr(LIGHT["window_intensity"])
    wl.CreateColorAttr(Gf.Vec3f(0.55, 0.72, 1.0))
    _place(wl.GetPrim(), (BAY_X[2], 0.0, -HALF + 0.005), (1, 1, 1), (180, 0, 0))  # emits +Z into the bay
    al = UsdLux.RectLight.Define(stage, f"{root}/AdjacentModule")
    al.CreateWidthAttr(1.2)
    al.CreateHeightAttr(0.15)
    al.CreateIntensityAttr(LIGHT["adjacent_intensity"])
    al.CreateEnableColorTemperatureAttr(True)
    al.CreateColorTemperatureAttr(4200.0)
    _place(al.GetPrim(), (X_MAX + 1.5, 0.0, HALF - 0.05), (1, 1, 1), (0, 0, 90))
    fl = UsdLux.DomeLight.Define(stage, f"{root}/AmbientFill")
    fl.CreateIntensityAttr(LIGHT["fill_intensity"])
    fl.CreateColorAttr(Gf.Vec3f(0.85, 0.9, 1.0))
    return root


CAMERA = {"eye": (-2.05, 0.50, 0.16), "target": (0.9, -0.30, 0.06), "focal_mm": 19.0, "h_aperture_mm": 36.0}


def look_at_xform(eye, target, up=(0, 0, 1)):
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
    return view.GetInverse()


def build_camera(stage):
    UsdGeom.Xform.Define(stage, "/World/Cameras")
    cam = UsdGeom.Camera.Define(stage, "/World/Cameras/Hero")
    cam.CreateFocalLengthAttr(CAMERA["focal_mm"])
    cam.CreateHorizontalApertureAttr(CAMERA["h_aperture_mm"])
    cam.CreateVerticalApertureAttr(CAMERA["h_aperture_mm"] * 9 / 16)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 60.0))
    cam.CreateFStopAttr(0.0)  # no depth of field
    x = UsdGeom.Xformable(cam.GetPrim())
    x.ClearXformOpOrder()
    x.AddTransformOp().Set(look_at_xform(CAMERA["eye"], CAMERA["target"]))
    return cam


# ---- R2 visual / kinematic actor --------------------------------------------------------------
def build_r2(stage, M, layout_targets):
    if not R2_USD.exists():
        raise SystemExit(f"Missing R2 import {R2_USD}; run scripts/make_r2_zero_g_derivative.py and the momentum-test importer first")
    src = Usd.Stage.Open(str(R2_USD))
    actor = UsdGeom.Xform.Define(stage, "/World/Actors/R2_Visual")
    roll, pitch, yaw = R2_ROOT["rpy_deg"]
    _place(actor.GetPrim(), R2_ROOT["pos"], (1, 1, 1), (roll, pitch, yaw))
    ref_path = "/World/Actors/R2_Visual/R2"
    r2 = stage.DefinePrim(ref_path, "Xform")
    rel = Path(R2_USD).relative_to(ROOT).as_posix()
    r2.GetReferences().AddReference("../" + rel)
    remap = lambda p: Sdf.Path(str(p).replace("/r2", ref_path, 1))
    # kinematic pose: child link orient = orient0 * Rz(q)   (all posed joints: axis Z, localRot1 = identity)
    joints = [j for j in src.Traverse() if j.IsA(UsdPhysics.RevoluteJoint)]
    J = {}

    def joint(key):
        if key not in J:
            match = [j for j in joints if key in j.GetName()]
            if len(match) != 1:
                raise SystemExit(f"pose key {key} matched {len(match)} joints")
            rj = UsdPhysics.RevoluteJoint(match[0])
            assert rj.GetAxisAttr().Get() == "Z" and rj.GetLocalRot1Attr().Get() == Gf.Quatf(1, 0, 0, 0)
            child = rj.GetBody1Rel().GetTargets()[0]
            o0 = src.GetPrimAtPath(child).GetAttribute("xformOp:orient").Get()
            J[key] = {"name": match[0].GetName(), "lo": rj.GetLowerLimitAttr().Get(), "hi": rj.GetUpperLimitAttr().Get(),
                      "attr": stage.GetPrimAtPath(remap(child)).GetAttribute("xformOp:orient"), "o0": o0, "q": 0.0}
        return J[key]

    def set_q(key, q):
        j = joint(key)
        if not (j["lo"] <= q <= j["hi"]):
            raise SystemExit(f"{key}={q} outside limits [{j['lo']}, {j['hi']}]")
        h = math.radians(q) / 2
        j["attr"].Set(j["o0"] * type(j["o0"])(math.cos(h), 0, 0, math.sin(h)))
        j["q"] = q

    for key, q in R2_POSE.items():
        set_q(key, q)
    link = lambda sub: [p for p in src.Traverse() if sub in p.GetName() and "tn__r2" + sub in p.GetName()][0].GetPath()
    actor_m = UsdGeom.Xformable(actor.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    ik_report = {}
    for chain, (keys, eff, frame, tgt) in R2_IK.items():
        eff_prim = stage.GetPrimAtPath(remap(link(eff)))
        target = Gf.Vec3d(*layout_targets[tgt]) if frame == "world" else actor_m.Transform(Gf.Vec3d(*tgt))
        def cost():
            p = UsdGeom.XformCache().GetLocalToWorldTransform(eff_prim).ExtractTranslation()
            return (p - target).GetLength() ** 2 + 2e-3 * sum(math.radians(joint(k)["q"]) ** 2 for k in keys)

        def descend():
            best = cost()
            for step in (24.0, 12.0, 6.0, 3.0, 1.5, 0.75):
                improved = True
                while improved:
                    improved = False
                    for k in keys:
                        q0 = joint(k)["q"]
                        for d in (step, -step):
                            q = min(max(q0 + d, joint(k)["lo"]), joint(k)["hi"])
                            set_q(k, q)
                            c = cost()
                            if c < best - 1e-9:
                                best, q0, improved = c, q, True
                        set_q(k, q0)
            return best

        # deterministic multi-start: each joint starts at 0 (clamped) or at 35 % of its range
        seeds = []
        for mask in range(2 ** len(keys)):
            s = []
            for i, k in enumerate(keys):
                j = joint(k)
                v = j["lo"] + 0.35 * (j["hi"] - j["lo"]) if mask >> i & 1 else 0.0
                s.append(min(max(v, j["lo"]), j["hi"]))
            seeds.append(s)
        best_c, best_q = 1e9, None
        for s in seeds:
            for k, v in zip(keys, s):
                set_q(k, v)
            c = descend()
            if c < best_c:
                best_c, best_q = c, [joint(k)["q"] for k in keys]
        for k, v in zip(keys, best_q):
            set_q(k, v)
        p = UsdGeom.XformCache().GetLocalToWorldTransform(eff_prim).ExtractTranslation()
        ik_report[chain] = {"residual_m": round((p - target).GetLength(), 4), "target_world": [round(v, 3) for v in target]}
    applied = {J[k]["name"]: round(J[k]["q"], 2) for k in J}
    # physics disabled for this actor: no articulation, no joints, kinematic bodies only
    for prim in src.Traverse():
        # use the authored listOp tokens, not GetAppliedSchemas(): outside Kit, Newton's schema is unregistered and
        # would be skipped, yet inside Kit NewtonArticulationRootAPI re-applies PhysicsArticulationRootAPI
        lop = prim.GetMetadata("apiSchemas")
        schemas = list(lop.ApplyOperations([])) if lop else []
        dst = stage.GetPrimAtPath(remap(prim.GetPath()))
        drop = [s for s in schemas if "ArticulationRootAPI" in s]
        if drop:
            lo = Sdf.TokenListOp()
            lo.deletedItems = drop
            dst.SetMetadata("apiSchemas", lo)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            dst.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)
    stage.GetPrimAtPath(f"{ref_path}/Physics").SetActive(False)
    # torso shell texture carries agency/company insignia: render a plain shell instead (no endorsement implied)
    insignia = []
    for prim in src.Traverse():
        if prim.IsInstance() and prim.GetName().startswith("Body_Cover"):
            dst = stage.GetPrimAtPath(remap(prim.GetPath()))
            dst.SetInstanceable(False)
            for m in Usd.PrimRange(dst):
                if m.IsA(UsdGeom.Mesh):
                    UsdShade.MaterialBindingAPI.Apply(m).Bind(M["r2_shell"], UsdShade.Tokens.strongerThanDescendants)
                    insignia.append(str(m.GetPath()))
    a = actor.GetPrim()
    a.CreateAttribute("iss:insigniaSuppressed", Sdf.ValueTypeNames.StringArray).Set(insignia)
    a.CreateAttribute("iss:ikResidual", Sdf.ValueTypeNames.String).Set(json.dumps(ik_report, sort_keys=True))
    a.CreateAttribute("iss:classification", Sdf.ValueTypeNames.Token).Set("KINEMATIC_VISUAL_ONLY")
    a.CreateAttribute("iss:physicsGate", Sdf.ValueTypeNames.String).Set(
        "NOT VALIDATED: R2 reset-safe zero-g Test B FAIL (docs/ISS_V0_MOMENTUM_GATE.md); pose is authored, not simulated")
    a.CreateAttribute("iss:sourceUrdf", Sdf.ValueTypeNames.String).Set(R2_URDF.relative_to(ROOT).as_posix())
    a.CreateAttribute("iss:sourceUrdfSha256", Sdf.ValueTypeNames.String).Set(sha256(R2_URDF) if R2_URDF.exists() else "UNKNOWN")
    a.CreateAttribute("iss:sourceUsd", Sdf.ValueTypeNames.String).Set(rel)
    a.CreateAttribute("iss:sourceUsdSha256", Sdf.ValueTypeNames.String).Set(sha256(R2_USD))
    a.CreateAttribute("iss:kinematicPoseDeg", Sdf.ValueTypeNames.String).Set(json.dumps(applied, sort_keys=True))
    return applied, ik_report, insignia


# ---- truth-layer switching (one VariantSet on /World, same camera) ----------------------------
LAYERS = {
    #             Visual  Collision  Affordances  R2      Task
    "visual":     (True,  False,     False,       True,   False),
    "collision":  (False, True,      False,       False,  False),
    "affordance": (False, True,      True,        False,  True),
}
LAYER_PRIMS = ["/World/Environment/ISS/Visual", "/World/Environment/ISS/Collision",
               "/World/Environment/ISS/Affordances", "/World/Actors/R2_Visual", "/World/Task"]


def build_variants(stage, M):
    world = stage.GetPrimAtPath("/World")
    vset = world.GetVariantSets().AddVariantSet("truthLayer")
    collision_gprims = [p.GetPath() for p in Usd.PrimRange(stage.GetPrimAtPath(LAYER_PRIMS[1])) if p.IsA(UsdGeom.Gprim)]
    for name, flags in LAYERS.items():
        vset.AddVariant(name)
        vset.SetVariantSelection(name)
        with vset.GetVariantEditContext():
            for path, on in zip(LAYER_PRIMS, flags):
                UsdGeom.Imageable(stage.GetPrimAtPath(path)).CreateVisibilityAttr().Set(
                    UsdGeom.Tokens.inherited if on else UsdGeom.Tokens.invisible)
            # all layer-specific opinions live inside the variants (a local opinion would always win over a variant)
            for p in collision_gprims:
                task = "/Handrail_" in str(p) or str(p).endswith("/MaintenancePanel")
                mat = M["context"] if name == "affordance" else (M["collision"] if task else M["collision_env"])
                UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(p)).Bind(mat)
    vset.SetVariantSelection("visual")


def main():
    SCENE.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(SCENE))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetMetadata("comment", "ISS Micro-Lab v0 (S1). ISS-INSPIRED authored geometry, not a digital twin. "
                                 "R2 is KINEMATIC / VISUAL ONLY. Built by scripts/build_iss_microlab.py")
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    for p in ("/World/Environment", "/World/Environment/ISS", "/World/Actors", "/World/Task"):
        UsdGeom.Xform.Define(stage, p)   # typed (imageable) so layer visibility inherits
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(0.0)   # microgravity (IVA); no orbital mechanics
    M = build_materials(stage)
    layout = {
        "handrails": {"Handrail_A": {"wall": "port", "x": -0.60, "u": -0.18},
                      "Handrail_B": {"wall": "starboard", "x": 0.80, "u": 0.30}},
        "panel": {"wall": "starboard", "x": 1.45, "u": -0.05, "sx": 0.42, "su": 0.30},
        "park": (0.78, -0.50, 0.08),
    }
    L, gw, gt, so, sw = handrail_dims()
    hb = layout["handrails"]["Handrail_B"]
    targets = {"handrail_B": wall_point(hb["wall"], hb["x"] - 0.05, hb["u"], -(so + gt / 2) - 0.36)}  # reach toward the grip, no contact authored
    build_visual(stage, M, layout)
    build_collision(stage, M, layout)
    build_affordances(stage, M, layout)
    build_lighting(stage)
    build_camera(stage)
    pose, ik, insignia = build_r2(stage, M, targets)
    build_variants(stage, M)
    stage.GetRootLayer().Save()
    text = SCENE.read_text(encoding="utf-8")
    SCENE.write_text(text.rstrip("\n") + "\n", encoding="utf-8", newline="\n")   # no trailing blank line (git diff --check)
    print(json.dumps({"scene": str(SCENE), "r2_pose_deg": pose, "ik": ik, "insignia_overrides": len(insignia),
                      "handrail_m": {"grip_len": L, "grip_section": [gw, gt], "standoff_height": so},
                      "aisle_m": 2 * HALF, "camera": CAMERA}, indent=1))


if __name__ == "__main__":
    main()

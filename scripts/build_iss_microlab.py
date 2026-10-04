"""Build the S1 ISS Micro-Lab scene (Visual / Collision / Affordance truth layers + kinematic R2 clip).

Pure pxr authoring; run with C:\\isaacsim\\python.bat scripts/build_iss_microlab.py
Visual = NASA US Lab interior + ISS handrail meshes (nasa/astrobee_media, Apache-2.0; converted by
scripts/import_iss_media.py). Collision = authored primitive proxies sized from the measured US Lab aisle.
R2 = referenced zero-g derivative, physics disabled, animated KINEMATICALLY (time-sampled joint orients).
"""
import hashlib
import json
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_microlab_v0.usda"
R2_USD = ROOT / "outputs" / "r2_derived" / "isaac_import" / "r2c6_zero_g" / "r2c6_zero_g.usda"
R2_URDF = ROOT / "outputs" / "r2_derived" / "r2c6_zero_g.urdf"
MEDIA = ROOT / "assets" / "cache" / "iss_media"

# ---- measured envelope of the US Lab mesh after USLAB_XFORM (metres; vertex-gap measurement, approximate) ------
HY, HZ = 1.00, 0.94        # clear aisle half-width (port/starboard rack faces) and half-height (overhead/deck)
CORNER_SUM = 1.55          # corner standoffs: free space satisfies |y| + |z| <= 1.55 (one corner measured, applied to all)
X_HALF = 3.30              # rack zone half-length; end cones/hatches beyond
HATCH_HALF = 0.635         # 1.27 m square hatchway measured at both module ends
USLAB_XFORM = {"translate": (4.2, 0.0, 0.0), "rotateXYZ": (90.0, 0.0, 0.0), "scale": 1.0858}  # 1.0858 as in astrobee_iss URDF
# NASA 30-inch handrail mesh bbox: length 0.784 m (incl. feet), depth 0.079 m from mount face, width 0.034 m
HANDRAIL = {"grip_len": 0.70, "grip_w": 0.034, "grip_t": 0.025, "standoff": 0.054, "standoff_w": 0.040, "mesh_len": 0.784}
LAYOUT = {
    "handrails": {"Handrail_A": {"wall": "port", "x": -1.70, "u": -0.20},
                  "Handrail_B": {"wall": "starboard", "x": 0.40, "u": 0.25}},
    "panel": {"wall": "starboard", "x": 1.25, "u": -0.15, "sx": 0.40, "su": 0.30},
    "park": (0.70, -0.35, 0.05),
}

# ---- kinematic R2 clip (visual storytelling only; never simulated) -------------------------------------------
FPS, DURATION, HERO_FRAME = 30, 8.0, 180
R2_FIXED = {"waistjoint0": 10.0, "neckjoint0": -12.0, "left_armjoint2": -60.0, "right_armjoint2": 60.0}
CHAINS = {
    "L": (["left_armjoint0", "left_armjoint1", "left_armjoint3", "left_armwristpitch"], "left_palm"),
    "R": (["right_armjoint0", "right_armjoint1", "right_armjoint3", "right_armwristpitch"], "right_palm"),
    "LL": (["left_legjoint0", "left_legjoint1", "left_legjoint3", "left_legjoint5"], "left_leg_foot"),
    "RL": (["right_legjoint0", "right_legjoint1", "right_legjoint3", "right_legjoint5"], "right_leg_foot"),
}
# glide head-first along the aisle, reach for Handrail_B, arrest and settle. Targets: ("body", xyz in R2 root frame)
# or ("grip", standoff distance inboard of Handrail_B grip in metres).
KEYS = [
    {"t": 0.0, "pos": (2.00, 0.00, 0.06), "rpy": (0.0, 52.0, -168.0), "L": ("body", (0.62, 0.22, 0.30)),
     "R": ("body", (0.60, -0.24, 0.26)), "LL": ("body", (-0.30, 0.15, -0.82)), "RL": ("body", (-0.34, -0.15, -0.80))},
    {"t": 2.5, "pos": (1.35, 0.02, 0.04), "rpy": (0.0, 48.0, -163.0), "L": ("body", (0.60, 0.25, 0.28)),
     "R": ("body", (0.58, -0.25, 0.24)), "LL": ("body", (-0.26, 0.16, -0.84)), "RL": ("body", (-0.36, -0.14, -0.78))},
    {"t": 4.5, "pos": (0.95, 0.02, 0.02), "rpy": (0.0, 40.0, -156.0), "L": ("grip", 0.18),
     "R": ("body", (0.50, -0.30, 0.20)), "LL": ("body", (-0.22, 0.17, -0.82)), "RL": ("body", (-0.32, -0.14, -0.72))},
    {"t": 6.0, "pos": (0.76, 0.03, 0.00), "rpy": (0.0, 34.0, -150.0), "L": ("grip", 0.13),
     "R": ("body", (0.36, -0.46, 0.10)), "LL": ("body", (-0.20, 0.17, -0.80)), "RL": ("body", (-0.30, -0.13, -0.66))},
    {"t": 8.0, "pos": (0.73, 0.03, -0.01), "rpy": (0.0, 32.0, -148.0), "L": ("grip", 0.13),
     "R": ("body", (0.32, -0.48, 0.08)), "LL": ("body", (-0.18, 0.17, -0.80)), "RL": ("body", (-0.28, -0.13, -0.64))},
]
CAMERA = {"eye": (-3.0, 0.45, 0.20), "target": (1.2, -0.25, -0.02), "focal_mm": 18.0, "h_aperture_mm": 36.0}
LIGHT = {"gla_intensity": 9000.0, "gla_kelvin": 4800.0, "fill_intensity": 40.0}


# ---- authoring helpers ------------------------------------------------------------------------------------
def material(stage, path, color, rough=0.6, metal=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _place(prim, center, scale=(1, 1, 1), rot_xyz_deg=(0, 0, 0)):
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    ops = [x.AddTranslateOp()]
    ops[0].Set(Gf.Vec3d(*center))
    ops.append(x.AddRotateXYZOp())
    ops[1].Set(Gf.Vec3f(*rot_xyz_deg))
    if tuple(scale) != (1, 1, 1):
        x.AddScaleOp().Set(Gf.Vec3f(*scale))
    return ops


def box(stage, path, center, size, mat=None, rot=(0, 0, 0)):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    _place(c.GetPrim(), center, size, rot)
    if mat:
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
    return c


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rel_asset(path):
    return "../" + Path(path).relative_to(ROOT).as_posix()   # relative to scenes/


WALLS = {"port": (2, +1, 1, HY), "starboard": (2, -1, 1, HY), "overhead": (1, +1, 2, HZ), "deck": (1, -1, 2, HZ)}


def wall_point(wall, x, u, depth):
    """Point on a rack face: x along module, u along the face, depth = offset from the face toward the hull."""
    in_ax, sign, n_ax, half = WALLS[wall]
    p = [x, 0.0, 0.0]
    p[n_ax] = sign * (half + depth)
    p[in_ax] = u
    return tuple(p)


def wall_size(wall, sx, su, sn):
    in_ax, _, n_ax, _ = WALLS[wall]
    s = [sx, 0.0, 0.0]
    s[in_ax] = su
    s[n_ax] = sn
    return tuple(s)


def corner_geom(sy, sz, thickness):
    """Centre and X-rotation of a 45-degree corner proxy lying on |y| + |z| = CORNER_SUM."""
    th = math.degrees(math.atan2(-sy, sz))
    off = thickness / 2 / math.sqrt(2)                       # push outward so the inner face lies on the line
    cy, cz = (HY + CORNER_SUM - HZ) / 2 + off, (CORNER_SUM - HY + HZ) / 2 + off
    width = math.sqrt(2) * (HY + HZ - CORNER_SUM)
    return (sy * cy, sz * cz), th, width


def look_at_xform(eye, target, up=(0, 0, 1)):
    return Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up)).GetInverse()


def smooth(a):
    return a * a * (3 - 2 * a)


# ---- Visual layer: NASA meshes (no authored decoration) ---------------------------------------------------
def build_materials(stage):
    m = "/World/Looks"
    UsdGeom.Scope.Define(stage, m)
    return {
        "r2_shell": material(stage, f"{m}/R2ShellPlain", (0.86, 0.86, 0.84), 0.45),
        "collision_env": material(stage, f"{m}/CollisionProxyBounds", (0.60, 0.61, 0.62), 0.85),
        "collision": material(stage, f"{m}/CollisionProxyTask", (0.90, 0.56, 0.20), 0.7),
        "context": material(stage, f"{m}/CollisionContext", (0.30, 0.31, 0.32), 0.9),
        "grasp": material(stage, f"{m}/AffordanceGraspable", (0.15, 0.65, 0.55), 0.5),
        "target": material(stage, f"{m}/AffordanceMaintenance", (0.30, 0.55, 0.95), 0.5),
        "waypoint": material(stage, f"{m}/AffordanceWaypoint", (0.92, 0.92, 0.92), 0.5),
    }


def handrail_mount(wall, x, u):
    """Mesh frame: length along local Z, protrudes along local +X from the mount face, width along local Y."""
    _, sign, n_ax, half = WALLS[wall]
    inward = [0.0, 0.0, 0.0]
    inward[n_ax] = -sign
    lx, lz = Gf.Vec3d(*inward), Gf.Vec3d(1, 0, 0)
    ly = Gf.Cross(lz, lx)
    m = Gf.Matrix4d(1.0)
    m.SetRow3(0, lx)
    m.SetRow3(1, ly)
    m.SetRow3(2, lz)
    m.SetTranslateOnly(Gf.Vec3d(*wall_point(wall, x, u, 0.0)))
    return m


def build_visual(stage):
    root = "/World/Environment/ISS/Visual"
    UsdGeom.Xform.Define(stage, root)
    lab = stage.DefinePrim(f"{root}/USLab", "Xform")
    lab.GetReferences().AddReference(rel_asset(MEDIA / "us_lab.usd"))
    _place(lab, USLAB_XFORM["translate"], (USLAB_XFORM["scale"],) * 3, USLAB_XFORM["rotateXYZ"])
    for name, spec in LAYOUT["handrails"].items():
        h = stage.DefinePrim(f"{root}/{name}", "Xform")
        h.GetReferences().AddReference(rel_asset(MEDIA / "handrail_30.usd"))
        x = UsdGeom.Xformable(h)
        x.ClearXformOpOrder()
        x.AddTransformOp().Set(handrail_mount(spec["wall"], spec["x"], spec["u"]))
    return root


# ---- Collision layer: authored primitive proxies sized from the measured aisle ---------------------------
def handrail_proxy(stage, root, name, wall, x, u):
    h = HANDRAIL
    p = f"{root}/{name}"
    UsdGeom.Xform.Define(stage, p)
    box(stage, f"{p}/Grip", wall_point(wall, x, u, -(h["standoff"] + h["grip_t"] / 2)),
        wall_size(wall, h["grip_len"], h["grip_w"], h["grip_t"]))
    foot = (h["mesh_len"] - h["standoff_w"]) / 2
    for k, dx in enumerate((-foot, foot)):
        box(stage, f"{p}/Standoff{k}", wall_point(wall, x + dx, u, -h["standoff"] / 2),
            wall_size(wall, h["standoff_w"], h["grip_w"], h["standoff"]))


def build_collision(stage):
    root = "/World/Environment/ISS/Collision"
    UsdGeom.Xform.Define(stage, root)
    L, t = 2 * X_HALF, 0.05
    slabs = {
        "PortWall": ((0, HY + t / 2, 0), (L, t, 2 * HZ)), "StarboardWall": ((0, -HY - t / 2, 0), (L, t, 2 * HZ)),
        "OverheadWall": ((0, 0, HZ + t / 2), (L, 2 * HY, t)), "DeckWall": ((0, 0, -HZ - t / 2), (L, 2 * HY, t)),
    }
    for end, sx in (("Aft", -1), ("Fwd", +1)):
        x0 = sx * (X_HALF + t / 2)
        for k, (cy, cz, ly, lz) in enumerate((
                (0, (HZ + HATCH_HALF) / 2, 2 * HY, HZ - HATCH_HALF), (0, -(HZ + HATCH_HALF) / 2, 2 * HY, HZ - HATCH_HALF),
                ((HY + HATCH_HALF) / 2, 0, HY - HATCH_HALF, 2 * HATCH_HALF), (-(HY + HATCH_HALF) / 2, 0, HY - HATCH_HALF, 2 * HATCH_HALF))):
            slabs[f"{end}Bulkhead{k}"] = ((x0, cy, cz), (t, ly, lz))
        tunnel = 4.2 - X_HALF   # hatchway/end-cone region simplified to a square tunnel, capped at the module end
        for k, (cy, cz, ly, lz) in enumerate(((0, HATCH_HALF + 0.01, 2 * HATCH_HALF, 0.02), (0, -HATCH_HALF - 0.01, 2 * HATCH_HALF, 0.02),
                                              (HATCH_HALF + 0.01, 0, 0.02, 2 * HATCH_HALF), (-HATCH_HALF - 0.01, 0, 0.02, 2 * HATCH_HALF))):
            slabs[f"{end}Hatchway{k}"] = ((sx * (X_HALF + tunnel / 2), cy, cz), (tunnel, ly, lz))
        slabs[f"{end}ScopeCap"] = ((sx * (4.2 + t / 2), 0, 0), (t, 2 * HATCH_HALF, 2 * HATCH_HALF))
    for name, (c, s) in slabs.items():
        box(stage, f"{root}/{name}", c, s)
    for sy in (+1, -1):
        for sz in (+1, -1):
            (cy, cz), th, width = corner_geom(sy, sz, t)
            tag = f"{'P' if sy > 0 else 'S'}{'O' if sz > 0 else 'D'}"
            box(stage, f"{root}/Standoff_{tag}", (0, cy, cz), (L, width, t), None, (th, 0, 0))
    for name, spec in LAYOUT["handrails"].items():
        handrail_proxy(stage, root, name, spec["wall"], spec["x"], spec["u"])
    tp = LAYOUT["panel"]
    box(stage, f"{root}/MaintenancePanel", wall_point(tp["wall"], tp["x"], tp["u"], -0.01),
        wall_size(tp["wall"], tp["sx"], tp["su"], 0.02))
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.IsA(UsdGeom.Gprim):
            UsdPhysics.CollisionAPI.Apply(prim)
    return root


# ---- Affordances + Task ----------------------------------------------------------------------------------
def _rel(prim, name, target):
    prim.CreateRelationship(name).SetTargets([Sdf.Path(target)])


def build_affordances(stage, M):
    root = "/World/Environment/ISS/Affordances"
    UsdGeom.Xform.Define(stage, root)
    h, pad = HANDRAIL, 0.004
    for name, spec in LAYOUT["handrails"].items():
        g = box(stage, f"{root}/{name}_Graspable", wall_point(spec["wall"], spec["x"], spec["u"], -(h["standoff"] + h["grip_t"] / 2)),
                wall_size(spec["wall"], h["grip_len"] - 0.06 + pad, h["grip_w"] + pad, h["grip_t"] + pad), M["grasp"]).GetPrim()
        g.CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set("graspable")
        g.CreateAttribute("iss:graspLength_m", Sdf.ValueTypeNames.Float).Set(h["grip_len"] - 0.06)
        g.CreateAttribute("iss:gripSection_m", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(h["grip_w"], h["grip_t"]))
        _rel(g, "iss:collisionProxy", f"/World/Environment/ISS/Collision/{name}/Grip")
        _rel(g, "iss:visualSource", f"/World/Environment/ISS/Visual/{name}")
    tp = LAYOUT["panel"]
    t = box(stage, f"{root}/MaintenancePanel_Target", wall_point(tp["wall"], tp["x"], tp["u"], -0.022),
            wall_size(tp["wall"], tp["sx"] + 0.01, tp["su"] + 0.01, 0.004), M["target"]).GetPrim()
    t.CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set("maintenance_target")
    _rel(t, "iss:collisionProxy", "/World/Environment/ISS/Collision/MaintenancePanel")
    _rel(t, "iss:visualSource", "/World/Environment/ISS/Visual/USLab")   # a rack-front region of the US Lab mesh
    UsdGeom.Xform.Define(stage, "/World/Task/Targets")
    tgt = UsdGeom.Xform.Define(stage, "/World/Task/Targets/MaintenancePanel").GetPrim()
    _place(tgt, wall_point(tp["wall"], tp["x"], tp["u"], -0.03))
    _rel(tgt, "iss:affordance", f"{root}/MaintenancePanel_Target")
    UsdGeom.Xform.Define(stage, "/World/Task/Waypoints")
    wp = UsdGeom.Sphere.Define(stage, "/World/Task/Waypoints/R2_Park")
    wp.CreateRadiusAttr(0.035)
    _place(wp.GetPrim(), LAYOUT["park"])
    UsdShade.MaterialBindingAPI.Apply(wp.GetPrim()).Bind(M["waypoint"])
    wp.GetPrim().CreateAttribute("iss:affordance", Sdf.ValueTypeNames.Token).Set("approach_park")
    _rel(wp.GetPrim(), "iss:graspCandidates", f"{root}/Handrail_B_Graspable")


# ---- Lighting / camera -----------------------------------------------------------------------------------
def build_lighting(stage):
    root = "/World/Environment/ISS/Lighting"
    UsdGeom.Xform.Define(stage, root)
    for sy in (+1, -1):
        (cy, cz), th, _ = corner_geom(sy, +1, -0.06)        # just inboard of the overhead corner standoffs
        for k, x in enumerate((-2.4, -0.8, 0.8, 2.4)):
            lt = UsdLux.RectLight.Define(stage, f"{root}/GLA_{'P' if sy > 0 else 'S'}{k}")
            lt.CreateWidthAttr(1.0)
            lt.CreateHeightAttr(0.12)
            lt.CreateIntensityAttr(LIGHT["gla_intensity"])
            lt.CreateEnableColorTemperatureAttr(True)
            lt.CreateColorTemperatureAttr(LIGHT["gla_kelvin"])
            _place(lt.GetPrim(), (x, cy, cz), (1, 1, 1), (th, 0, 0))
    fl = UsdLux.SphereLight.Define(stage, f"{root}/AisleFill")
    fl.CreateRadiusAttr(0.3)
    fl.CreateIntensityAttr(LIGHT["fill_intensity"])
    fl.CreateColorAttr(Gf.Vec3f(0.92, 0.95, 1.0))
    _place(fl.GetPrim(), (-2.2, 0.0, 0.3))


def build_camera(stage):
    UsdGeom.Xform.Define(stage, "/World/Cameras")
    cam = UsdGeom.Camera.Define(stage, "/World/Cameras/Hero")
    cam.CreateFocalLengthAttr(CAMERA["focal_mm"])
    cam.CreateHorizontalApertureAttr(CAMERA["h_aperture_mm"])
    cam.CreateVerticalApertureAttr(CAMERA["h_aperture_mm"] * 9 / 16)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 60.0))
    cam.CreateFStopAttr(0.0)
    x = UsdGeom.Xformable(cam.GetPrim())
    x.ClearXformOpOrder()
    x.AddTransformOp().Set(look_at_xform(CAMERA["eye"], CAMERA["target"]))


# ---- R2: visual actor, physics disabled, kinematic clip ---------------------------------------------------
class R2Rig:
    """Joint-orient rig on the referenced R2 USD: child-link orient = orient0 * Rz(q) (posed joints: axis Z, localRot1 = I)."""

    def __init__(self, stage, src, ref_path, actor):
        self.stage, self.src, self.ref, self.actor = stage, src, ref_path, actor
        self.joints = [j for j in src.Traverse() if j.IsA(UsdPhysics.RevoluteJoint)]
        self.J = {}
        a = UsdGeom.Xformable(actor.GetPrim())
        a.ClearXformOpOrder()
        self.t_op, self.r_op = a.AddTranslateOp(), a.AddRotateXYZOp()

    def remap(self, p):
        return Sdf.Path(str(p).replace("/r2", self.ref, 1))

    def joint(self, key):
        if key not in self.J:
            match = [j for j in self.joints if key in j.GetName()]
            if len(match) != 1:
                raise SystemExit(f"pose key {key} matched {len(match)} joints")
            rj = UsdPhysics.RevoluteJoint(match[0])
            assert rj.GetAxisAttr().Get() == "Z" and rj.GetLocalRot1Attr().Get() == Gf.Quatf(1, 0, 0, 0)
            child = rj.GetBody1Rel().GetTargets()[0]
            self.J[key] = {"name": match[0].GetName(), "lo": rj.GetLowerLimitAttr().Get(), "hi": rj.GetUpperLimitAttr().Get(),
                           "attr": self.stage.GetPrimAtPath(self.remap(child)).GetAttribute("xformOp:orient"),
                           "o0": self.src.GetPrimAtPath(child).GetAttribute("xformOp:orient").Get(), "q": 0.0}
        return self.J[key]

    def quat(self, key, q):
        j = self.joint(key)
        h = math.radians(q) / 2
        return j["o0"] * type(j["o0"])(math.cos(h), 0, 0, math.sin(h))

    def set_q(self, key, q, time=Usd.TimeCode.Default()):
        j = self.joint(key)
        if not (j["lo"] - 1e-6 <= q <= j["hi"] + 1e-6):
            raise SystemExit(f"{key}={q} outside limits [{j['lo']}, {j['hi']}]")
        j["attr"].Set(self.quat(key, q), time)
        if time == Usd.TimeCode.Default():
            j["q"] = q

    def set_root(self, pos, rpy, time=Usd.TimeCode.Default()):
        self.t_op.Set(Gf.Vec3d(*pos), time)
        self.r_op.Set(Gf.Vec3f(*rpy), time)

    def link(self, sub):
        p = [x for x in self.src.Traverse() if x.GetName().startswith("tn__r2" + sub)][0]
        return self.stage.GetPrimAtPath(self.remap(p.GetPath()))

    def solve(self, keys, eff, target, multistart):
        """Coordinate-descent IK on default-time values; deterministic multi-start for the first key only."""
        eff_prim = self.link(eff)

        def cost():
            p = UsdGeom.XformCache().GetLocalToWorldTransform(eff_prim).ExtractTranslation()
            return (p - target).GetLength() ** 2 + 2e-3 * sum(math.radians(self.joint(k)["q"]) ** 2 for k in keys)

        def descend():
            best = cost()
            for step in (24.0, 12.0, 6.0, 3.0, 1.5, 0.75):
                improved = True
                while improved:
                    improved = False
                    for k in keys:
                        q0, j = self.joint(k)["q"], self.joint(k)
                        for d in (step, -step):
                            q = min(max(q0 + d, j["lo"]), j["hi"])
                            self.set_q(k, q)
                            c = cost()
                            if c < best - 1e-9:
                                best, q0, improved = c, q, True
                        self.set_q(k, q0)
            return best

        seeds = [[self.joint(k)["q"] for k in keys]]
        if multistart:
            seeds = []
            for mask in range(2 ** len(keys)):
                seeds.append([min(max(self.joint(k)["lo"] + 0.35 * (self.joint(k)["hi"] - self.joint(k)["lo"])
                                      if mask >> i & 1 else 0.0, self.joint(k)["lo"]), self.joint(k)["hi"]) for i, k in enumerate(keys)])
        best_c, best_q = 1e9, None
        for s in seeds:
            for k, v in zip(keys, s):
                self.set_q(k, v)
            c = descend()
            if c < best_c:
                best_c, best_q = c, [self.joint(k)["q"] for k in keys]
        for k, v in zip(keys, best_q):
            self.set_q(k, v)
        p = UsdGeom.XformCache().GetLocalToWorldTransform(eff_prim).ExtractTranslation()
        return round((p - target).GetLength(), 4)


def build_r2(stage, M):
    if not R2_USD.exists():
        raise SystemExit(f"Missing R2 import {R2_USD}; run scripts/make_r2_zero_g_derivative.py and the momentum-test importer first")
    src = Usd.Stage.Open(str(R2_USD))
    actor = UsdGeom.Xform.Define(stage, "/World/Actors/R2_Visual")
    ref_path = "/World/Actors/R2_Visual/R2"
    stage.DefinePrim(ref_path, "Xform").GetReferences().AddReference(rel_asset(R2_USD))
    rig = R2Rig(stage, src, ref_path, actor)
    for k, q in R2_FIXED.items():
        rig.set_q(k, q)
    hb, h = LAYOUT["handrails"]["Handrail_B"], HANDRAIL
    solved, residual = [], {}
    for i, key in enumerate(KEYS):
        rig.set_root(key["pos"], key["rpy"])
        m = UsdGeom.Xformable(actor.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for chain, (keys, eff) in CHAINS.items():
            kind, val = key[chain]
            target = (m.Transform(Gf.Vec3d(*val)) if kind == "body" else
                      Gf.Vec3d(*wall_point(hb["wall"], hb["x"] - 0.05, hb["u"], -(h["standoff"] + h["grip_t"] / 2) - val)))
            residual[f"t{key['t']}:{chain}"] = rig.solve(keys, eff, target, multistart=(i == 0))
        solved.append({k: rig.joint(k)["q"] for k in rig.J})

    times = [k["t"] * FPS for k in KEYS]

    def pose_at(f):
        s = max(i for i in range(len(KEYS) - 1) if times[i] <= f) if f < times[-1] else len(KEYS) - 2
        a = smooth(min(1.0, (f - times[s]) / (times[s + 1] - times[s])))
        A, B = KEYS[s], KEYS[s + 1]
        lerp = lambda u, v: tuple((1 - a) * x + a * y for x, y in zip(u, v))
        return lerp(A["pos"], B["pos"]), lerp(A["rpy"], B["rpy"]), {k: (1 - a) * solved[s][k] + a * solved[s + 1][k] for k in rig.J}

    for f in range(int(DURATION * FPS) + 1):          # one time sample per frame: plays on the Isaac Sim timeline
        pos, rpy, q = pose_at(f)
        rig.set_root(pos, rpy, Usd.TimeCode(f))
        for k, v in q.items():
            rig.set_q(k, v, Usd.TimeCode(f))
    pos, rpy, q = pose_at(HERO_FRAME)                    # default (non-animated) value = hero frame
    rig.set_root(pos, rpy)
    for k, v in q.items():
        rig.set_q(k, v)

    # physics disabled: no articulation root (authored listOp tokens, so Kit-registered Newton schemas are caught too)
    for prim in src.Traverse():
        lop = prim.GetMetadata("apiSchemas")
        drop = [s for s in (lop.ApplyOperations([]) if lop else []) if "ArticulationRootAPI" in s]
        dst = stage.GetPrimAtPath(rig.remap(prim.GetPath()))
        if drop:
            lo = Sdf.TokenListOp()
            lo.deletedItems = drop
            dst.SetMetadata("apiSchemas", lo)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            dst.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)
    stage.GetPrimAtPath(f"{ref_path}/Physics").SetActive(False)
    insignia = []   # torso texture carries agency/company insignia: plain shell in this scene (asset files untouched)
    for prim in src.Traverse():
        if prim.IsInstance() and prim.GetName().startswith("Body_Cover"):
            dst = stage.GetPrimAtPath(rig.remap(prim.GetPath()))
            dst.SetInstanceable(False)
            for mesh in Usd.PrimRange(dst):
                if mesh.IsA(UsdGeom.Mesh):
                    UsdShade.MaterialBindingAPI.Apply(mesh).Bind(M["r2_shell"], UsdShade.Tokens.strongerThanDescendants)
                    insignia.append(str(mesh.GetPath()))
    a = actor.GetPrim()
    attrs = {
        "iss:classification": (Sdf.ValueTypeNames.Token, "KINEMATIC_VISUAL_ONLY"),
        "iss:physicsGate": (Sdf.ValueTypeNames.String, "NOT VALIDATED: R2 reset-safe zero-g Test B FAIL "
                            "(docs/ISS_V0_MOMENTUM_GATE.md); motion is an authored keyframe clip, not a simulation"),
        "iss:clip": (Sdf.ValueTypeNames.String, f"{DURATION:.0f} s kinematic keyframe clip at {FPS} fps, timecodes 0-{int(DURATION * FPS)}; "
                     f"hero/default pose = frame {HERO_FRAME}"),
        "iss:sourceUrdf": (Sdf.ValueTypeNames.String, R2_URDF.relative_to(ROOT).as_posix()),
        "iss:sourceUrdfSha256": (Sdf.ValueTypeNames.String, sha256(R2_URDF) if R2_URDF.exists() else "UNKNOWN"),
        "iss:sourceUsd": (Sdf.ValueTypeNames.String, R2_USD.relative_to(ROOT).as_posix()),
        "iss:sourceUsdSha256": (Sdf.ValueTypeNames.String, sha256(R2_USD)),
        "iss:ikResidual": (Sdf.ValueTypeNames.String, json.dumps(residual, sort_keys=True)),
    }
    for n, (typ, v) in attrs.items():
        a.CreateAttribute(n, typ).Set(v)
    a.CreateAttribute("iss:insigniaSuppressed", Sdf.ValueTypeNames.StringArray).Set(insignia)
    return residual, insignia


# ---- truth-layer switching (one VariantSet on /World, same camera) ---------------------------------------
LAYERS = {
    #             Visual  Collision  Affordances  R2      Task
    "visual":     (True,  False,     False,       True,   False),
    "collision":  (False, True,      False,       False,  False),
    "affordance": (False, True,      True,        False,  True),
}
LAYER_PRIMS = ["/World/Environment/ISS/Visual", "/World/Environment/ISS/Collision",
               "/World/Environment/ISS/Affordances", "/World/Actors/R2_Visual", "/World/Task"]


def build_variants(stage, M):
    vset = stage.GetPrimAtPath("/World").GetVariantSets().AddVariantSet("truthLayer")
    col = [p.GetPath() for p in Usd.PrimRange(stage.GetPrimAtPath(LAYER_PRIMS[1])) if p.IsA(UsdGeom.Gprim)]
    for name, flags in LAYERS.items():
        vset.AddVariant(name)
        vset.SetVariantSelection(name)
        with vset.GetVariantEditContext():   # every layer-specific opinion lives inside the variant
            for path, on in zip(LAYER_PRIMS, flags):
                UsdGeom.Imageable(stage.GetPrimAtPath(path)).CreateVisibilityAttr().Set(
                    UsdGeom.Tokens.inherited if on else UsdGeom.Tokens.invisible)
            for p in col:
                task = "/Handrail_" in str(p) or str(p).endswith("/MaintenancePanel")
                mat = M["context"] if name == "affordance" else (M["collision"] if task else M["collision_env"])
                UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(p)).Bind(mat)
    vset.SetVariantSelection("visual")


def main():
    for f in (MEDIA / "us_lab.usd", MEDIA / "handrail_30.usd"):
        if not f.exists():
            raise SystemExit(f"Missing {f}; run scripts/import_iss_media.py first")
    SCENE.parent.mkdir(parents=True, exist_ok=True)
    if SCENE.exists():
        SCENE.unlink()
    stage = Usd.Stage.CreateNew(str(SCENE))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(FPS)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(int(DURATION * FPS))
    stage.SetMetadata("comment", "ISS Micro-Lab (S1). Visual: NASA US Lab interior + ISS handrail meshes (nasa/astrobee_media, "
                                 "Apache-2.0). Collision/affordance: authored proxies. R2 is KINEMATIC / VISUAL ONLY. "
                                 "Built by scripts/build_iss_microlab.py")
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    for p in ("/World/Environment", "/World/Environment/ISS", "/World/Actors", "/World/Task"):
        UsdGeom.Xform.Define(stage, p)
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(0.0)   # microgravity (IVA); no orbital mechanics
    M = build_materials(stage)
    build_visual(stage)
    build_collision(stage)
    build_affordances(stage, M)
    build_lighting(stage)
    build_camera(stage)
    residual, insignia = build_r2(stage, M)
    build_variants(stage, M)
    stage.GetRootLayer().Save()
    text = SCENE.read_text(encoding="utf-8")
    SCENE.write_text(text.rstrip("\n") + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"scene": str(SCENE), "ik_residual_m": residual, "insignia_overrides": len(insignia),
                      "aisle_m": [2 * HY, 2 * HZ], "frames": int(DURATION * FPS), "camera": CAMERA}, indent=1))


if __name__ == "__main__":
    main()

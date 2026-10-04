"""S1.2 shared layout: G1 physics actor and free-floating cargo added to the S1 ISS scene (pxr only).

Used by scripts/s12_record_physics.py (PhysX run) and scripts/s12_render_panels.py (replay + render).
"""
from pathlib import Path

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_microlab_v0.usda"
OUT = ROOT / "outputs" / "s12"
GATE_CONFIG = ROOT / "configs" / "iss_v0_momentum_v3.json"
G1_URDF = Path(r"C:\Users\getch\Documents\Codex\2026-09-24\robotfit-assets\unitree_ros\robots\g1_description\g1_29dof.urdf")
FPS, DURATION = 30, 10.0

G1_PATH = "/World/Actors/G1_Physics"
G1_PLACE = {"translate": (-1.05, 0.0, 0.10), "rotateXYZ": (0.0, 6.0, 15.0)}   # floating upright, facing +x
CARGO_PATH = "/World/Actors/Cargo"
BAG_SIZE = (0.50, 0.43, 0.25)   # CTB-sized generic box; not a NASA cargo model
BAG_MASS = 10.0
BAGS = [  # position (m), linear velocity (m/s), angular velocity (deg/s): authored initial conditions
    {"pos": (1.00, 0.35, 0.40), "vel": (0.18, -0.10, -0.12), "ang": (10.0, 25.0, 5.0)},
    {"pos": (2.00, -0.40, -0.30), "vel": (-0.15, 0.20, 0.10), "ang": (-20.0, 5.0, 15.0)},
    {"pos": (1.60, 0.30, -0.45), "vel": (0.05, -0.22, 0.25), "ang": (5.0, -15.0, 30.0)},
]
RESTITUTION, FRICTION = 0.2, 0.5
PANELS = [  # (key, truth layer, camera eye, target, focal mm)
    ("r2", "visual", (-3.0, 0.45, 0.20), (1.2, -0.25, -0.02), 15.0),
    ("g1", "visual", (0.25, 0.55, 0.30), (-1.05, 0.0, 0.0), 15.0),
    ("cargo", "collision", (-0.75, 0.55, 0.55), (1.60, -0.10, -0.10), 14.0),
]
PANEL_W, PANEL_H = 640, 1080


def place(prim, translate, rotate, scale=None):
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*translate))
    x.AddRotateXYZOp().Set(Gf.Vec3f(*rotate))
    if scale is not None:
        x.AddScaleOp().Set(Gf.Vec3f(*scale))


def add_g1(stage, g1_usd):
    """Reference the imported G1 USD (its default prim) at the floating pose."""
    prim = stage.DefinePrim(G1_PATH, "Xform")
    prim.GetReferences().AddReference(str(g1_usd))
    place(prim, G1_PLACE["translate"], G1_PLACE["rotateXYZ"])
    return prim


def add_cargo(stage, physics):
    mat = UsdShade.Material.Define(stage, "/World/Looks/CargoBag")
    sh = UsdShade.Shader.Define(stage, "/World/Looks/CargoBag/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.80, 0.78, 0.71))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdGeom.Xform.Define(stage, CARGO_PATH)
    paths = []
    if physics:
        pm = UsdShade.Material.Define(stage, "/World/Looks/CargoPhysics")
        api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
        api.CreateRestitutionAttr(RESTITUTION)
        api.CreateStaticFrictionAttr(FRICTION)
        api.CreateDynamicFrictionAttr(FRICTION)
    for i, b in enumerate(BAGS):
        cube = UsdGeom.Cube.Define(stage, f"{CARGO_PATH}/Bag_{i}")
        cube.CreateSizeAttr(1.0)
        x = UsdGeom.Xformable(cube)
        x.AddTranslateOp().Set(Gf.Vec3d(*b["pos"]))
        x.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))
        x.AddScaleOp().Set(Gf.Vec3f(*BAG_SIZE))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)
        if physics:
            rb = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            rb.CreateVelocityAttr(Gf.Vec3f(*b["vel"]))
            rb.CreateAngularVelocityAttr(Gf.Vec3f(*b["ang"]))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(BAG_MASS)
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
                pm, UsdShade.Tokens.weakerThanDescendants, "physics")
        paths.append(str(cube.GetPath()))
    return paths


def add_collision_groups(stage):
    """G1 never collides with the environment or cargo (same no-contact condition as the gate); cargo hits proxies."""
    groups = {}
    for name, include in (("G1", G1_PATH), ("Env", "/World/Environment/ISS/Collision"), ("Cargo", CARGO_PATH)):
        g = UsdPhysics.CollisionGroup.Define(stage, f"/World/CollisionGroups/{name}")
        g.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(include)
        groups[name] = g
    groups["G1"].CreateFilteredGroupsRel().SetTargets([groups["Env"].GetPath(), groups["Cargo"].GetPath()])


def look_at(eye, target):
    return Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()

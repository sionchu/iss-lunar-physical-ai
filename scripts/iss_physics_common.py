"""S1.3 shared layout for the ISS physics clip (pxr only; import after SimulationApp starts).

Two PhysX G1 instances in the S1 ISS scene: G1_Hold (free-floating posture hold + gate Test B arm motion) and
G1_Grasp (posture hold, reaches Handrail_B; the grasp is a scripted fixed-joint constraint, not finger contact).
R2 is shown only as the S1.1 kinematic clip. Used by iss_physics_record.py and iss_physics_render.py.
"""
from pathlib import Path

from pxr import Gf, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_microlab_v0.usda"
OUT = ROOT / "outputs" / "iss_physics"
GATE_CONFIG = ROOT / "configs" / "iss_v0_momentum_v3.json"
G1_URDF = Path(r"C:\Users\getch\Documents\Codex\2026-09-24\robotfit-assets\unitree_ros\robots\g1_description\g1_29dof.urdf")
FPS, DURATION = 30, 10.0

HOLD_PATH, GRASP_PATH = "/World/Actors/G1_Hold", "/World/Actors/G1_Grasp"
HOLD_PLACE = {"translate": (-1.05, 0.0, 0.10), "rotateXYZ": (0.0, 6.0, 15.0)}
GRASP_YAW = -90.0                      # G1 faces the starboard wall (-y)
RAIL = "/World/Environment/ISS/Affordances/Handrail_B_Graspable"
HAND = "right_rubber_hand"
REACH_DOFS = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint"]
REACH_TARGET_PELVIS = (0.40, -0.16, 0.22)   # desired hand origin in the pelvis frame (IK seed target)
HAND_STANDOFF = 0.06                   # hand-link origin this far aisle-side of the grip axis (palm mesh extends beyond it)
REACH_T = (0.5, 2.0)                   # s: arm moves from the default pose to the reach pose
APPROACH_V = 0.02                      # m/s initial drift toward the wall
CAPTURE_R = 0.012                      # m: grasp created when the hand is within this of its planned grip pose
HOLD_GAINS = (100.0, 10.0)             # posture-hold PD on every DOF (same gains as gate Test B drives)

PANELS = [  # (key, truth layer, eye, target, focal mm)
    ("r2", "visual", (-3.0, 0.45, 0.20), (1.2, -0.25, -0.02), 15.0),
    ("hold", "visual", (0.25, 0.55, 0.30), (-1.05, 0.0, 0.0), 15.0),
    ("grasp", "visual", (1.75, -0.30, 0.40), (0.35, -0.72, 0.10), 16.0),
]
PANEL_W, PANEL_H = 640, 1080


def place(prim, translate, rotate):
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*translate))
    x.AddRotateXYZOp().Set(Gf.Vec3f(*rotate))


def add_g1(stage, path, g1_usd, translate, rotate):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(str(g1_usd))
    place(prim, translate, rotate)
    return prim


def add_collision_groups(stage):
    """Both G1s are filtered from the environment proxies and each other: no contacts anywhere (gate condition)."""
    env = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/Env")
    env.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget("/World/Environment/ISS/Collision")
    for name, path in (("G1_Hold", HOLD_PATH), ("G1_Grasp", GRASP_PATH)):
        g = UsdPhysics.CollisionGroup.Define(stage, f"/World/CollisionGroups/{name}")
        g.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(path)
        g.CreateFilteredGroupsRel().SetTargets([env.GetPath(), "/World/CollisionGroups/G1_Hold",
                                                "/World/CollisionGroups/G1_Grasp"])


def rail_segment(stage):
    """Grip axis of Handrail_B from its affordance box: center, unit axis, half length, aisle-side normal."""
    m = UsdGeom.Xformable(stage.GetPrimAtPath(RAIL)).ComputeLocalToWorldTransform(0)
    c = m.ExtractTranslation()
    rows = [m.GetRow3(i) for i in range(3)]
    k = max(range(3), key=lambda i: rows[i].GetLength())
    axis = rows[k].GetNormalized()
    normal = Gf.Vec3d(0, 1 if c[1] < 0 else -1, 0)          # toward the aisle centre
    return c, axis, rows[k].GetLength() / 2.0, normal


def look_at(eye, target):
    return Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()

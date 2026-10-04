"""S1.4 shared layout for the ISS worksite clip (pxr only; import after SimulationApp starts).

One PhysX G1 performs an IVA worksite task: drift to Handrail_B, anchor with the right hand (scripted fixed-joint
grasp, not finger contact), then reach the left hand to the maintenance panel (no contact). R2 appears only as the
S1.1 kinematic clip. Used by iss_physics_record.py and iss_physics_render.py.
"""
from pathlib import Path

from pxr import Gf, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_microlab_v0.usda"
OUT = ROOT / "outputs" / "iss_physics"
GATE_CONFIG = ROOT / "configs" / "iss_v0_momentum_v3.json"
G1_URDF = Path(r"C:\Users\getch\Documents\Codex\2026-09-24\robotfit-assets\unitree_ros\robots\g1_description\g1_29dof.urdf")
FPS, DURATION = 30, 10.0

G1_PATH = "/World/Actors/G1"
G1_YAW = -90.0                          # faces the starboard wall (-y); its left side points to +x (panel side)
RAIL = "/World/Environment/ISS/Affordances/Handrail_B_Graspable"
PANEL = "/World/Environment/ISS/Affordances/MaintenancePanel_Target"
GRIP_S = 0.30                           # m along the rail axis from its centre (toward the panel)
PANEL_UV = (-0.19, 0.13)                # m on the panel face from its centre (along x, along z)
STANDOFF = 0.06                         # hand-link origin offset from the rail axis / panel face (palm extends beyond)
ARMS = {
    "right": {"hand": "right_rubber_hand", "target_pelvis": (0.30, -0.16, 0.32), "t": (0.5, 2.0),
              "dofs": ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                       "right_elbow_joint"]},
    "left": {"hand": "left_rubber_hand", "t": (3.0, 5.0),
             "dofs": ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
                      "left_elbow_joint"]},
}
APPROACH_V = 0.035                      # m/s initial drift toward the wall (start pose is feed-forward corrected)
CAPTURE_R = 0.04                        # m: grasp created at the first closest approach within this of the plan
POSTURE_GAINS = (100.0, 10.0)           # PD on every DOF toward the posture targets

PANELS = [  # (key, truth layer, eye, target, focal mm)
    ("r2", "visual", (-3.0, 0.45, 0.20), (1.2, -0.25, -0.02), 15.0),
    ("g1", "visual", (-0.55, -0.20, 0.45), (0.95, -0.78, 0.02), 15.0),
    ("aff", "affordance", (1.65, -0.35, 0.65), (0.85, -0.88, 0.0), 16.0),
]
PANEL_W, PANEL_H = 640, 1080


def add_g1(stage, g1_usd, translate):
    prim = stage.DefinePrim(G1_PATH, "Xform")
    prim.GetReferences().AddReference(str(g1_usd))
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*translate))
    x.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, G1_YAW))
    return prim


def add_collision_groups(stage):
    """G1 is filtered against the environment proxies: no contacts anywhere (gate condition)."""
    env = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/Env")
    env.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget("/World/Environment/ISS/Collision")
    g = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/G1")
    g.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(G1_PATH)
    g.CreateFilteredGroupsRel().SetTargets([env.GetPath(), g.GetPath()])


def box_frame(stage, path):
    """World centre and the three scaled axes of an affordance box (unit cube)."""
    m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)
    return m.ExtractTranslation(), [m.GetRow3(i) for i in range(3)]


def worksite_points(stage):
    """Planned right-hand point (rail) and left-hand point (panel), both STANDOFF toward the aisle."""
    rc, ra = box_frame(stage, RAIL)
    pc, pa = box_frame(stage, PANEL)
    n = Gf.Vec3d(0, 1 if rc[1] < 0 else -1, 0)
    axis = max(ra, key=lambda v: v.GetLength())
    rail_pt = rc + axis.GetNormalized() * GRIP_S + n * STANDOFF
    panel_pt = pc + Gf.Vec3d(PANEL_UV[0], 0, PANEL_UV[1]) + n * STANDOFF
    return {"rail_centre": rc, "rail_axis": axis.GetNormalized(), "rail_half": axis.GetLength() / 2,
            "rail_pt": rail_pt, "panel_pt": panel_pt, "normal": n}


def look_at(eye, target):
    return Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()

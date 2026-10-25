"""S1.4 shared layout for the ISS worksite clip (pxr only; import after SimulationApp starts).

One PhysX G1 performs an IVA worksite task: drift to Handrail_B, anchor with the right hand (scripted fixed-joint
grasp, not finger contact), then reach the left hand to the maintenance panel (no contact). R2 appears only as the
S1.1 kinematic clip. Used by iss_physics_record.py and iss_physics_render.py.
"""
import os
from pathlib import Path

from pxr import Gf, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "iss_microlab_v0.usda"
OUT = ROOT / "outputs" / "iss_physics"
GATE_CONFIG = ROOT / "configs" / "iss_v0_momentum_v3.json"
# Canonical G1 (2026-10-16): g1_29dof_rev_1_0.urdf from unitreerobotics/unitree_ros @ 5994d4f (BSD-3-Clause), sparse
# checkout in gitignored assets/raw/unitree_ros_5994d4f/; override with ISS_G1_URDF. Expected SHA-256 c0ae739c...
# The former g1_29dof.urdf (ccfc6fd, e1dc8936..., marked Deprecated upstream) is kept for history only.
G1_URDF = Path(os.environ.get("ISS_G1_URDF", ROOT / "assets" / "raw" / "unitree_ros_5994d4f" / "robots" / "g1_description"
                              / "g1_29dof_rev_1_0.urdf"))
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


def add_g1_self_colliders(stage, g1, env_collision_path):
    """Collision setup for G1 runs with self-collision on.

    The URDF gives the rubber hands no collision shape, so a hand could sink into the head or torso: each hand gets
    a convex hull of its visual mesh, which touches G1 links only (filtered against env_collision_path: the scripted
    grasp places the hand on the rail) and not its own wrist chain (the hull overlaps it by construction). Two pairs
    two joints apart overlap by design and otherwise stay in permanent contact (a stiff wrist stalled main step 4 in
    sc1_main): wrist_yaw|wrist_roll and pelvis_contour|hip_pitch; shoulder_roll|torso is filtered per collider. Filters are applied on the leaf-side body and only
    to same-limb targets: a FilteredPairsAPI on a body acted on its whole USD subtree (filters on pelvis/waist or on a
    shoulder against its head sibling switched off every arm-torso and hand-head contact, run sc_test8 vs sc_test7).
    Pelvis and waist_yaw_link get hull colliders as well, in a collision group (see below). Mass is unchanged
    (explicit URDF inertials). Returns the collider mesh paths.
    """
    from pxr import Usd
    hg = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/G1Hands")
    ig = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/EnvForG1Hands")
    ig.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(env_collision_path)
    hg.CreateFilteredGroupsRel().SetTargets([ig.GetPath()])
    bodies = {p.GetName(): p for p in Usd.PrimRange(g1) if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    added = []
    for side in ("left", "right"):
        p = bodies[f"{side}_rubber_hand"]
        p.SetInstanceable(False)
        for c in p.GetChildren():                      # the visual meshes sit in instanced children
            if c.IsInstance():
                c.SetInstanceable(False)
        for m in Usd.PrimRange(p):
            if m.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(m)
                UsdPhysics.MeshCollisionAPI.Apply(m).CreateApproximationAttr("convexHull")
                hg.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(m.GetPath())
                added.append(str(m.GetPath()))
        fp = UsdPhysics.FilteredPairsAPI.Apply(p)
        for n in ("wrist_yaw", "wrist_pitch", "wrist_roll", "elbow"):
            fp.GetFilteredPairsRel().AddTarget(bodies[f"{side}_{n}_link"].GetPath())
        UsdPhysics.FilteredPairsAPI.Apply(bodies[f"{side}_wrist_yaw_link"]).GetFilteredPairsRel().AddTarget(
            bodies[f"{side}_wrist_roll_link"].GetPath())
        UsdPhysics.FilteredPairsAPI.Apply(bodies["pelvis_contour_link"]).GetFilteredPairsRel().AddTarget(
            bodies[f"{side}_hip_pitch_link"].GetPath())
    # shoulder_roll's URDF collider is a 3 cm cylinder at the joint housing, inside the torso shell whenever the arm
    # moves inward (19k contacts held the right arm stiff in sc3_jem step 4; cuRobo ignores the pair as well):
    # filtered collider-to-collider, so the filter cannot spread over the arm subtree
    torso = bodies["torso_link"]
    tsub = [o for o in bodies.values() if o != torso and o.GetPath().HasPrefix(torso.GetPath())]
    for c in torso.GetChildren():
        if c.IsInstance() and not any(c.GetPath().HasPrefix(o.GetPath()) for o in tsub):
            c.SetInstanceable(False)
    torso_cols = [m.GetPath() for m in Usd.PrimRange(torso) if m.HasAPI(UsdPhysics.CollisionAPI)
                  and not any(m.GetPath().HasPrefix(o.GetPath()) for o in tsub)]
    roll_cols = []
    for side in ("left", "right"):
        roll = bodies[f"{side}_shoulder_roll_link"]
        rsub = [o for o in bodies.values() if o != roll and o.GetPath().HasPrefix(roll.GetPath())]
        for m in Usd.PrimRange(roll):
            if m.HasAPI(UsdPhysics.CollisionAPI) and not any(m.GetPath().HasPrefix(o.GetPath()) for o in rsub):
                fr = UsdPhysics.FilteredPairsAPI.Apply(m).GetFilteredPairsRel()
                for t in torso_cols:
                    fr.AddTarget(t)
                roll_cols.append(m.GetPath())
    if not torso_cols or len(roll_cols) < 2:
        raise RuntimeError(f"shoulder/torso collider prims not found: {roll_cols}, {torso_cols}")
    # pelvis and waist_yaw_link have visual meshes only too (a wrist then sank into the waist, sc_test9): convex hulls
    # of those, in a collision group that ignores the legs, the torso (and head, logo) and the ISS but not the arms;
    # groups act on the listed colliders, the torso include excludes both arm subtrees
    cg = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/G1Core")
    ng = UsdPhysics.CollisionGroup.Define(stage, "/World/CollisionGroups/G1NonArm")
    inc = ng.GetCollidersCollectionAPI()
    for n in ("pelvis_contour_link", "left_hip_pitch_link", "right_hip_pitch_link", "torso_link"):
        inc.CreateIncludesRel().AddTarget(bodies[n].GetPath())
    for side in ("left", "right"):
        inc.CreateExcludesRel().AddTarget(bodies[f"{side}_shoulder_pitch_link"].GetPath())
    cg.CreateFilteredGroupsRel().SetTargets([ng.GetPath(), ig.GetPath()])
    n_hands = len(added)
    for name in ("pelvis", "waist_yaw_link"):
        p = bodies[name]
        sub = [o for o in bodies.values() if o != p and o.GetPath().HasPrefix(p.GetPath())]
        for c in p.GetChildren():
            if c.IsInstance() and not any(c.GetPath().HasPrefix(o.GetPath()) for o in sub):
                c.SetInstanceable(False)
        for m in Usd.PrimRange(p):
            if m.IsA(UsdGeom.Mesh) and not any(m.GetPath().HasPrefix(o.GetPath()) for o in sub):
                UsdPhysics.CollisionAPI.Apply(m)
                UsdPhysics.MeshCollisionAPI.Apply(m).CreateApproximationAttr("convexHull")
                cg.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(m.GetPath())
                added.append(str(m.GetPath()))
    if n_hands < 2 or len(added) < 4:
        raise RuntimeError(f"collider meshes not found: {added}")
    return added


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

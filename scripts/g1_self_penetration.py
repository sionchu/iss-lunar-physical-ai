"""G1 self-penetration of a recorded ISS run, measured on the link meshes (what the viewer sees).

python scripts/g1_self_penetration.py outputs/iss_activity/main_sv5_run.json [--every 15] [--out ..._selfpen.json]
Link world poses come from the run record (frames.poses, the same poses the renderer replays). Each link's mesh is
the canonical URDF's mesh for that link (visual = collision STL for every G1 link that has one; rubber hands use
their visual STL). For every pair of links more than two joints apart in the URDF tree (closer links overlap by
design: joint housings, e.g. waist_yaw|torso, knee|ankle_roll) and not one hand against its own wrist chain, the depth is the deepest vertex of one mesh inside the convex hull of the other
(both directions), the same convex approximation PhysX uses for dynamic colliders. The hull is built from at most
120 of the mesh's hull vertices (slightly inside the full hull: tends to under-report), while concave links (torso)
are over-approximated by any hull (over-reports there).
Reports per pair the worst depth and the fraction of sampled frames above 0.5 / 1 / 2 cm.
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets" / "raw" / "unitree_ros_5994d4f" / "robots" / "g1_description" / "g1_29dof_rev_1_0.urdf"
WRIST = ("wrist_yaw", "wrist_pitch", "wrist_roll", "elbow")      # filtered against the hand collider in the runs


def load_links():
    r = ET.parse(URDF).getroot()
    meshes, adj = {}, set()
    for ln in r.findall("link"):
        geo = [g for g in ln.findall("collision") + ln.findall("visual") if g.find("geometry/mesh") is not None]
        if not geo:
            continue
        g = geo[0]
        m = trimesh.load(URDF.parent / g.find("geometry/mesh").get("filename"), force="mesh")
        o = g.find("origin")
        if o is not None:
            T = np.eye(4)
            T[:3, :3] = Rotation.from_euler("xyz", [float(v) for v in o.get("rpy", "0 0 0").split()]).as_matrix()
            T[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
            m.apply_transform(T)
        v = m.vertices if len(m.vertices) <= 400 else m.vertices[np.random.default_rng(0).choice(len(m.vertices), 400,
                                                                                                    replace=False)]
        hv = m.vertices[ConvexHull(m.vertices).vertices]
        if len(hv) > 120:                                     # coarser hull (subset of hull vertices): faster, and
            hv = hv[np.random.default_rng(1).choice(len(hv), 120, replace=False)]   # slightly smaller, so a lower bound
        hull = ConvexHull(hv)
        c = m.vertices.mean(axis=0)
        meshes[ln.get("name")] = (np.asarray(v), hull.equations, c, float(np.linalg.norm(m.vertices - c, axis=1).max()))
    nb = {}
    for j in r.findall("joint"):
        a, b = j.find("parent").get("link"), j.find("child").get("link")
        nb.setdefault(a, set()).add(b)
        nb.setdefault(b, set()).add(a)
    for a in nb:                                               # one or two joints apart
        for b in nb[a]:
            adj.add(frozenset((a, b)))
            for c in nb[b] - {a}:
                adj.add(frozenset((a, c)))
    for s in ("left", "right"):
        for w in WRIST:
            adj.add(frozenset((f"{s}_rubber_hand", f"{s}_{w}_link")))
    return meshes, adj


def depth(pts, eq):
    """Deepest point inside the convex hull (halfspaces n.x + d <= 0); 0 if none is inside."""
    s = pts @ eq[:, :3].T + eq[:, 3]
    inside = (s <= 0).all(axis=1)
    return float((-s[inside].max(axis=1)).max()) if inside.any() else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run", type=Path)
    ap.add_argument("--every", type=int, default=15)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    meshes, adj = load_links()
    rec = json.loads(args.run.read_text(encoding="utf-8"))
    names = [p.split("/")[-1] for p in rec["link_paths"]]
    idx = [k for k, n in enumerate(names) if n in meshes]
    pairs = [(a, b) for i, a in enumerate(idx) for b in idx[i + 1:] if frozenset((names[a], names[b])) not in adj]
    worst, cnt, n = {}, {}, 0
    tot = {"gt_5mm": 0, "gt_1cm": 0, "gt_2cm": 0}
    for f in range(0, len(rec["frames"]["t"]), args.every):
        pos, quat = rec["frames"]["poses"][f]
        world = {}
        for k in idx:
            R = Rotation.from_quat(np.r_[quat[k][1:], quat[k][0]])
            v, eq, c, rad = meshes[names[k]]
            Rm = R.as_matrix()
            n_w = eq[:, :3] @ Rm.T                                # hull planes into world
            d_w = eq[:, 3] - n_w @ np.asarray(pos[k])
            world[k] = (v @ Rm.T + pos[k], np.c_[n_w, d_w], Rm @ c + pos[k], rad)
        frame_max = 0.0
        for a, b in pairs:
            if np.linalg.norm(world[a][2] - world[b][2]) > world[a][3] + world[b][3]:
                continue                                       # bounding spheres apart
            d = max(depth(world[a][0], world[b][1]), depth(world[b][0], world[a][1]))
            if d > 0:
                key = f"{names[a]}|{names[b]}"
                worst[key] = max(worst.get(key, 0.0), d)
                c = cnt.setdefault(key, [0, 0, 0])
                for j, th in enumerate((0.005, 0.01, 0.02)):
                    c[j] += d > th
                frame_max = max(frame_max, d)
        for k, th in (("gt_5mm", 0.005), ("gt_1cm", 0.01), ("gt_2cm", 0.02)):
            tot[k] += frame_max > th
        n += 1
    res = {"run": args.run.name, "self_collision": rec.get("self_collision", False), "sampled": n, "every": args.every,
           "frames_penetrating": {k: round(v / max(n, 1), 4) for k, v in tot.items()},
           "pairs": {k: {"max_m": round(worst[k], 4), "frac_gt_5mm": round(cnt[k][0] / n, 4),
                         "frac_gt_1cm": round(cnt[k][1] / n, 4), "frac_gt_2cm": round(cnt[k][2] / n, 4)}
                     for k in sorted(worst, key=lambda x: -cnt[x][1]) if cnt[k][0]},
           "method": "link meshes (URDF) at recorded poses; vertex depth inside the other link's convex hull"}
    txt = json.dumps(res, indent=1)
    if args.out:
        args.out.write_text(txt, encoding="utf-8")
    print(txt[:2500])


if __name__ == "__main__":
    main()

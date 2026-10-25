"""G1 self-collision with cuRobo's sphere model: which link pairs overlap, at a pose or along a recorded run.

<curobo-venv>\\Scripts\\python.exe scripts/g1_self_collision.py [--run outputs/iss_activity/main_sv5_run.json]
    [--every 3] [--out outputs/iss_activity/main_sv5_selfcol.json]
Robot model as in plan_iss_activity.robot_cfg() (legs locked in the planner tuck). Pairs listed in the config's
self_collision_ignore (adjacent links) are skipped and self_collision_buffer is added to each sphere radius, as
cuRobo's self-collision cost does. Without --run, reports the
overlapping pairs at the planner's rest pose (all actuated joints 0, legs tucked); with --run, the per-pair worst
overlap and the fraction of sampled frames with any overlap above 0 / 1 cm / 3 cm. Overlap = r_a + r_b - |c_a - c_b|
(sphere proxies, conservative by construction; not the PhysX collision meshes).
"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch
from curobo._src.state.state_joint import JointState

from plan_iss_activity import Planner, base_seed, robot_cfg, rot


def pair_mask(cfg, links):
    ign = cfg["kinematics"].get("self_collision_ignore") or {}
    skip = {(a, b) for a, bs in ign.items() for b in bs}
    skip |= {(b, a) for a, b in skip}
    n = len(links)
    m = np.zeros((n, n), bool)
    for i in range(n):
        for j in range(i + 1, n):
            m[i, j] = links[i] != links[j] and (links[i], links[j]) not in skip
    return m


def overlaps(pl, mask, links, qvec, buf):
    ks = pl.ik.compute_kinematics(JointState.from_position(torch.as_tensor(qvec, device="cuda")[None],
                                                            joint_names=pl.joints))
    sp = ks.robot_spheres.reshape(-1, 4).cpu().numpy()
    valid = sp[:, 3] > 0
    c, r = sp[:, :3], sp[:, 3] + buf                  # self_collision_buffer per link, as cuRobo applies it
    d = np.linalg.norm(c[:, None] - c[None], axis=-1)
    ov = r[:, None] + r[None] - d
    ov[~(mask & valid[:, None] & valid[None])] = -1.0
    out = {}
    for i, j in zip(*np.nonzero(ov > 0)):
        k = tuple(sorted((links[i], links[j])))
        out[k] = max(out.get(k, 0.0), float(ov[i, j]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", type=Path)
    ap.add_argument("--every", type=int, default=3)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    cfg = robot_cfg()
    pl = Planner(use_scene=False)
    links = np.asarray(pl.sphere_link)
    mask = pair_mask(cfg, links)
    sb = cfg["kinematics"].get("self_collision_buffer") or {}
    buf = np.array([sb.get(n, 0.0) for n in links], np.float32)
    if not args.run:
        q = np.array([0.0] * len(pl.joints), np.float32)
        res = {"pose": "rest (actuated 0, legs tucked)",
               "pairs": {f"{a}|{b}": round(v, 4) for (a, b), v in sorted(overlaps(pl, mask, links, q, buf).items(),
                                                                       key=lambda x: -x[1])}}
    else:
        f = json.loads(args.run.read_text(encoding="utf-8"))["frames"]
        worst, frames, counts, n = {}, {}, {"any": 0, "gt_1cm": 0, "gt_3cm": 0}, 0
        for i in range(0, len(f["t"]), args.every):
            (pp, pq), q = f["pelvis"][i], f["q"][i]
            base = base_seed(pp, rot(pq))
            o = overlaps(pl, mask, links, np.array([base[j] if j in base else q.get(j, 0.0) for j in pl.joints],
                                                   np.float32), buf)
            n += 1
            m = max(o.values(), default=0.0)
            for k, th in (("any", 0.0), ("gt_1cm", 0.01), ("gt_3cm", 0.03)):
                counts[k] += m > th
            for k, v in o.items():
                worst[k] = max(worst.get(k, 0.0), v)
                frames[k] = frames.get(k, 0) + 1
        res = {"run": args.run.name, "sampled": n, "every": args.every,
               "frames_overlapping": {k: round(v / max(n, 1), 4) for k, v in counts.items()},
               "pairs": {f"{a}|{b}": {"max_m": round(v, 4), "frame_frac": round(frames[(a, b)] / n, 4)}
                         for (a, b), v in sorted(worst.items(), key=lambda x: -frames[x[0]])}}
    res["note"] = "cuRobo sphere proxies; adjacent pairs from self_collision_ignore skipped; legs at the planner tuck"
    txt = json.dumps(res, indent=1)
    if args.out:
        args.out.write_text(txt, encoding="utf-8")
    print(txt[:4000])


if __name__ == "__main__":
    main()

"""Lunar v0 evaluation: one trained G1 velocity policy walks the fixed waypoint course; metrics vs the approved gates.

Run with Isaac Lab's Python from the Isaac Lab root (it imports Isaac Lab's rsl_rl helpers):
  C:\\dev\\IsaacLab\\_isaac_sim\\python.bat <repo>/scripts/lunar_walk_eval.py --checkpoint <model_*.pt>
      --gravity 1.62 --seeds 1000 1001 ... --out <repo>/outputs/lunar_v0/<name>_eval.json [--headless]
Task: Isaac-Velocity-Flat-G1-Play-v0 (Isaac Lab's G1 asset; see docs/LUNAR_V0_PLAN.md section 6a), one environment per
evaluation seed, gravity (0, 0, -g). Course from the start pose: 10 m straight, 90 deg left turn, 2 m, stop 2 s, 2 m.
A waypoint follower turns heading error into the policy's velocity command (vx, wz).
Gates (approved 2026-10-23, docs/LUNAR_V0_PLAN.md section 5), all fixed before this script ran:
  success  every waypoint within 0.3 m, no fall, within the time limit   >= 9/10 seeds
  fall     pelvis height < 0.45 m, tilt > 60 deg, or a base-contact termination   0 on passing seeds
  slip     horizontal foot speed while that foot is in contact (> 1 N)   median < 0.05 m/s
  tracking commanded vs measured base planar velocity, RMS               < 0.15 m/s
  torque   steps with any joint |torque| >= 99 % of its effort limit     < 5 %
  energy   sum |torque * joint speed| dt per metre walked                 recorded only
"""
import argparse
import json
import math
import sys
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--checkpoint", required=True, help="model_*.pt, or a run directory (its last model_N.pt)")
parser.add_argument("--gravity", type=float, default=1.62)
parser.add_argument("--seeds", type=int, nargs="+", default=list(range(1000, 1010)))
parser.add_argument("--time-limit", type=float, default=90.0)
parser.add_argument("--out", type=Path, required=True)
ISAACLAB = Path(r"C:\dev\IsaacLab")
sys.path.insert(0, str(ISAACLAB / "scripts" / "reinforcement_learning" / "rsl_rl"))
from isaaclab_tasks.utils import add_launcher_args  # noqa: E402

add_launcher_args(parser)
args = parser.parse_args()
sys.argv = [sys.argv[0]]
if Path(args.checkpoint).is_dir():
    args.checkpoint = str(max(Path(args.checkpoint).glob("model_*.pt"), key=lambda f: int(f.stem.split("_")[1])))

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import isaaclab_tasks  # noqa: E402,F401
import importlib.metadata as metadata  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402
from isaaclab_tasks.utils import launch_simulation  # noqa: E402
from isaaclab_tasks.utils.hydra import hydra_task_config  # noqa: E402

TASK = "Isaac-Velocity-Flat-G1-Play-v0"
GATES = {"success_min": 9, "slip_median_max": 0.05, "tracking_rms_max": 0.15, "torque_frac_max": 0.05}
COURSE = [(10.0, 0.0, 0.0), (10.0, 2.0, 2.0), (10.0, 4.0, 0.0)]   # (x, y, stop seconds) in the start frame


def T(x):
    """Isaac Lab data fields are warp-backed ProxyArrays: read them as torch tensors explicitly."""
    return x.torch if hasattr(x, "torch") else x


def run_seed(seed, env_cfg, agent_cfg):
    env_cfg.seed = seed
    env = gym.make(TASK, cfg=env_cfg)
    venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(venv, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(args.checkpoint)
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    u = env.unwrapped
    robot, contact = u.scene["robot"], u.scene["contact_forces"]
    cmd_term = u.command_manager.get_term("base_velocity")
    dt = u.step_dt
    obs = venv.get_observations()
    feet = robot.find_bodies(".*ankle_roll_link")[0]
    feet_c = contact.find_bodies(".*ankle_roll_link")[0]
    lim = T(robot.data.joint_effort_limits)[0].abs().cpu().numpy()
    p0 = T(robot.data.root_pos_w)[0, :2].cpu().numpy().copy()
    yaw0 = float(T(robot.data.heading_w)[0])
    R0 = np.array([[math.cos(yaw0), -math.sin(yaw0)], [math.sin(yaw0), math.cos(yaw0)]])
    wps = [(p0 + R0 @ np.array([x, y]), stop) for x, y, stop in COURSE]
    k, stop_left, t = 0, None, 0.0
    slip, track_err, torque_hits, steps, energy, dist = [], [], 0, 0, 0.0, 0.0
    joint_hits = np.zeros(len(lim), int)
    fall, reached, prev = None, 0, p0.copy()
    while t < args.time_limit and k < len(wps):
        pos = T(robot.data.root_pos_w)[0, :2].cpu().numpy()
        yaw = float(T(robot.data.heading_w)[0])
        goal, stop = wps[k]
        d = goal - pos
        dist_to = float(np.linalg.norm(d))
        if stop_left is not None:
            vx, wz = 0.0, 0.0
            stop_left -= dt
            if stop_left <= 0:
                stop_left, k = None, k + 1
        elif dist_to < 0.3:
            reached += 1
            if stop > 0:
                stop_left = stop
                vx, wz = 0.0, 0.0
            else:
                k += 1
                continue
        else:
            err = math.atan2(d[1], d[0]) - yaw
            err = math.atan2(math.sin(err), math.cos(err))
            wz = float(np.clip(1.5 * err, -1.0, 1.0))
            vx = float(np.clip(0.8 * dist_to, 0.0, 0.8)) * (1.0 if math.cos(err) > 0.7 else 0.15)
        cmd_term.vel_command_b[:] = torch.tensor([[vx, 0.0, wz]], device=u.device)
        with torch.inference_mode():
            obs, _, dones, extras = venv.step(policy(obs))
        cmd_term.vel_command_b[:] = torch.tensor([[vx, 0.0, wz]], device=u.device)
        t += dt
        steps += 1
        h = float(T(robot.data.root_pos_w)[0, 2])
        g = T(robot.data.projected_gravity_b)[0].cpu().numpy()
        tilt = math.degrees(math.acos(float(np.clip(-g[2], -1.0, 1.0))))
        if bool(dones[0]) or h < 0.45 or tilt > 60.0:
            fall = {"t_s": round(t, 2), "pelvis_height_m": round(h, 3), "tilt_deg": round(tilt, 1),
                    "terminated": bool(dones[0])}
            break
        vb = T(robot.data.root_lin_vel_b)[0, :2].cpu().numpy()
        track_err.append(float(np.linalg.norm(vb - np.array([vx, 0.0]))))
        fz = T(contact.data.net_forces_w)[0, feet_c, 2].abs().cpu().numpy()
        fv = T(robot.data.body_lin_vel_w)[0, feet, :2].cpu().numpy()
        for c, v in zip(fz, fv):
            if c > 1.0:
                slip.append(float(np.linalg.norm(v)))
        tau = T(robot.data.applied_torque)[0].cpu().numpy()
        qd = T(robot.data.joint_vel)[0].cpu().numpy()
        hit = np.abs(tau) >= 0.99 * lim
        torque_hits += int(np.any(hit))
        joint_hits += hit
        energy += float(np.abs(tau * qd).sum()) * dt
        dist += float(np.linalg.norm(pos - prev))
        prev = pos.copy()
    env.close()
    success = fall is None and k >= len(wps)
    return {"seed": seed, "success": success, "waypoints_reached": reached, "fall": fall, "sim_time_s": round(t, 2),
            "slip_median_m_s": round(float(np.median(slip)), 4) if slip else None,
            "tracking_rms_m_s": round(float(np.sqrt(np.mean(np.square(track_err)))), 4) if track_err else None,
            "torque_limit_frac": round(torque_hits / max(steps, 1), 4),
            "torque_limit_by_joint": {robot.joint_names[j]: {"frac": round(int(joint_hits[j]) / max(steps, 1), 4),
                                                            "effort_limit": round(float(lim[j]), 2)}
                                      for j in np.argsort(-joint_hits)[:6] if joint_hits[j] > 0},
            "energy_J_per_m": round(energy / max(dist, 1e-6), 2), "distance_m": round(dist, 2)}


@hydra_task_config(TASK, "rsl_rl_cfg_entry_point")   # resolves the physics/renderer presets, as play does
def main(env_cfg, agent_cfg):
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))   # as Isaac Lab's play does
    env_cfg.scene.num_envs = 1
    env_cfg.sim.gravity = (0.0, 0.0, -args.gravity)
    env_cfg.episode_length_s = args.time_limit + 10.0
    c = env_cfg.commands.base_velocity
    c.resampling_time_range = (1e6, 1e6)
    c.heading_command = False
    c.rel_standing_envs = 0.0
    results = []
    with launch_simulation(env_cfg, args):
        for seed in args.seeds:
            results.append(run_seed(seed, env_cfg, agent_cfg))
            print(json.dumps(results[-1]), flush=True)
            write(results)            # inside the context: closing the Kit app ends the process (first run lost it)


def write(results):
    ok = [r for r in results if r["success"]]

    def med(key):
        v = [r[key] for r in ok if r[key] is not None]
        return float(np.median(v)) if v else None
    summary = {"checkpoint": args.checkpoint, "task": TASK, "gravity_m_s2": args.gravity, "seeds": args.seeds,
               "seeds_done": len(results), "course_start_frame_m": COURSE, "gates": GATES, "successes": len(ok),
               "falls_on_success": sum(r["fall"] is not None for r in ok),
               "slip_median_m_s": med("slip_median_m_s"), "tracking_rms_m_s": med("tracking_rms_m_s"),
               "torque_limit_frac": med("torque_limit_frac"), "energy_J_per_m": med("energy_J_per_m")}
    summary["pass"] = bool(summary["successes"] >= GATES["success_min"]
                           and summary["slip_median_m_s"] is not None and summary["slip_median_m_s"] < GATES["slip_median_max"]
                           and summary["tracking_rms_m_s"] is not None and summary["tracking_rms_m_s"] < GATES["tracking_rms_max"]
                           and summary["torque_limit_frac"] is not None and summary["torque_limit_frac"] < GATES["torque_frac_max"])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"summary": summary, "per_seed": results}, indent=1), encoding="utf-8")
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()

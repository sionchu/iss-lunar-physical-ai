"""Playback-speed map for a recorded ISS run (speed ramp for the showcase).

python scripts/speed_map.py outputs/iss_activity/calm2_main_run.json --out outputs/showcase_v6/main_speed.json
    [--start 0 --end 1e9] [--max-screen-deg-s 90 --fast 20] | [--capture 2 --move 10 --idle 24 ...]
Two modes, both from the run record only:
- --max-screen-deg-s W (v6): the speed of every frame is the largest of LEVELS that keeps the pelvis turning at most
  W deg/s on screen (pelvis angular speed smoothed over 0.5 s), capped at --fast. Body and arms then play on one
  clock: v5's phase rule (slow at captures, fast in moves) showed the body turning at a median 190 deg/s on screen
  because nearly all body rotation happens during moves. Runs shorter than --min-hold s on screen are merged into
  the next run (keeping the slower speed) so the badge stays readable.
- phase rule (v5): CAPTURE x around each (or every Nth) capture and the panel reach, MOVE x in moves, IDLE x
  otherwise.
Output: [[t0, t1, speed], ...] covering [start, end], for render_iss_activity.py --speed-map, and the screen length.
"""
import argparse
import json
from pathlib import Path

import numpy as np

LEVELS = (1, 1.5, 2, 3, 4, 6, 8, 12, 16, 20, 24)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=1e9)
    ap.add_argument("--max-screen-deg-s", type=float)
    ap.add_argument("--fast", type=float, default=20.0)
    ap.add_argument("--min-hold", type=float, default=2.5)
    ap.add_argument("--capture", type=float, default=2.0)
    ap.add_argument("--move", type=float, default=10.0)
    ap.add_argument("--idle", type=float, default=24.0)
    ap.add_argument("--pre", type=float, default=1.2)
    ap.add_argument("--post", type=float, default=0.6)
    ap.add_argument("--slow-every", type=int, default=1, help="slow down only at every Nth capture (and the last)")
    args = ap.parse_args()
    rec = json.loads(args.run.read_text(encoding="utf-8"))
    t, ph = np.asarray(rec["frames"]["t"]), rec["frames"]["phase"]
    end = min(args.end, t[-1])
    sel = (t >= args.start) & (t <= end)
    if args.max_screen_deg_s:
        from scipy.ndimage import uniform_filter1d
        from scipy.spatial.transform import Rotation as R
        rot = R.from_quat(np.asarray([q for _, q in rec["frames"]["pelvis"]])[:, [1, 2, 3, 0]])
        w = np.r_[0.0, np.degrees((rot[1:] * rot[:-1].inv()).magnitude()) / np.diff(t)]
        w = uniform_filter1d(w, 15)                          # 0.5 s at 30 Hz
        want = np.minimum(args.fast, args.max_screen_deg_s / np.maximum(w, 1e-6))
        sp = np.array([max([v for v in LEVELS if v <= x] or [LEVELS[0]]) for x in want])
    else:
        caps = [s["t_s"] for s in rec["segments"] if s.get("status") == "CAPTURED" and s.get("t_s")]
        caps = [c for k, c in enumerate(caps) if k % args.slow_every == args.slow_every - 1 or k == len(caps) - 1]
        sp = np.array([args.capture if p == "panel_reach" or any(c - args.pre <= x <= c + args.post for c in caps)
                       else (args.move if p == "move" else args.idle) for x, p in zip(t, ph)])
    t, sp = t[sel], sp[sel]
    raw = []
    for x, v in zip(t, sp):
        if raw and raw[-1][2] == v:
            raw[-1][1] = float(x)
        else:
            raw.append([float(x), float(x), float(v)])
    out = []                                                 # greedy: grow a run until it lasts min-hold on
    for a, b, v in raw:                                      # screen, at the slowest speed it absorbed
        if out and (out[-1][1] - out[-1][0]) / out[-1][2] < args.min_hold:
            out[-1] = [out[-1][0], b, min(out[-1][2], v)]
        else:
            out.append([a, b, v])
    if len(out) > 1 and (out[-1][1] - out[-1][0]) / out[-1][2] < args.min_hold:
        a, b, v = out.pop()
        out[-1] = [out[-1][0], b, min(out[-1][2], v)]
    screen = sum((b - a) / v for a, b, v in out)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out), encoding="utf-8")
    print(json.dumps({"segments": len(out), "sim_s": round(float(end - args.start), 1), "screen_s": round(screen, 1)}))


if __name__ == "__main__":
    main()

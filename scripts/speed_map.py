"""Playback-speed map for a recorded ISS run (speed ramp for the showcase): slow at captures, fast in transit.

python scripts/speed_map.py outputs/iss_activity/calm2_main_run.json --out outputs/showcase_v5/main_speed.json
    [--start 0 --end 1e9] [--capture 2 --move 10 --idle 24 --pre 1.2 --post 0.6]
Speeds come from the run record only: CAPTURE x from pre s before to post s after each (or every Nth) capture and over
the panel reach; IDLE x over hold / settle / retry / servo (little visible motion); MOVE x over planned moves. Output:
[[t0, t1, speed], ...] covering [start, end], for render_iss_activity.py --speed-map, plus the resulting screen
length at 30 fps.
"""
import argparse
import json
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=1e9)
    ap.add_argument("--capture", type=float, default=2.0)
    ap.add_argument("--move", type=float, default=10.0)
    ap.add_argument("--idle", type=float, default=24.0)
    ap.add_argument("--pre", type=float, default=1.2)
    ap.add_argument("--post", type=float, default=0.6)
    ap.add_argument("--slow-every", type=int, default=1, help="slow down only at every Nth capture (and the last)")
    args = ap.parse_args()
    rec = json.loads(args.run.read_text(encoding="utf-8"))
    t, ph = rec["frames"]["t"], rec["frames"]["phase"]
    end = min(args.end, t[-1])
    caps = [s["t_s"] for s in rec["segments"] if s.get("status") == "CAPTURED" and s.get("t_s")]
    caps = [c for k, c in enumerate(caps) if k % args.slow_every == args.slow_every - 1 or k == len(caps) - 1]

    def speed(x, p):
        if p == "panel_reach" or any(c - args.pre <= x <= c + args.post for c in caps):
            return args.capture
        return args.move if p == "move" else args.idle

    out = []
    for x, p in zip(t, ph):
        if not args.start <= x <= end:
            continue
        v = speed(x, p)
        if out and out[-1][2] == v:
            out[-1][1] = x
        else:
            out.append([x, x, v])
    screen = sum((b - a) / v for a, b, v in out)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out), encoding="utf-8")
    print(json.dumps({"segments": len(out), "sim_s": round(end - args.start, 1), "screen_s": round(screen, 1)}))


if __name__ == "__main__":
    main()

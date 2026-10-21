"""Compose the microgravity vs ground comparison clip with ffmpeg (no simulation; inputs are rendered clips).

python scripts/compose_gravity_comparison.py [--zero outputs/iss_activity/fin_main_0g.mp4]
    [--ground outputs/iss_activity/fin_main_1g.mp4] [--gui outputs/iss_activity/gui_gravity_raw.mp4]
Layout 1920x1620: top row = 0 g (left) | 9.81 m/s^2 (right), each 960x540, same route, planner and controller;
bottom = the Isaac Sim GUI clip of the PhysicsScene gravity switch (1920x1080). The shorter top clip holds its last
frame; the GUI clip loops. Labels name the condition of each panel. Output: outputs/iss_activity/iss_gravity_comparison.mp4.
"""
import argparse
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "iss_activity"
FONT = "C\\:/Windows/Fonts/segoeui.ttf"


def duration(p):
    r = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "json", str(p)],
                       capture_output=True, text=True, check=True)
    return float(json.loads(r.stdout)["format"]["duration"])


def label(text, size, y):
    t = text.replace(":", "\\:").replace("'", "")
    return (f"drawtext=fontfile='{FONT}':text='{t}':fontsize={size}:fontcolor=white:box=1:boxcolor=black@0.55:"
            f"boxborderw=8:x=(w-text_w)/2:y={y}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--zero", type=Path, default=OUT / "fin_main_0g.mp4")
    ap.add_argument("--ground", type=Path, default=OUT / "fin_main_1g.mp4")
    ap.add_argument("--gui", type=Path, default=OUT / "gui_gravity_raw.mp4")
    ap.add_argument("--out", type=Path, default=OUT / "iss_gravity_comparison.mp4")
    args = ap.parse_args()
    if not shutil.which("ffmpeg"):
        raise SystemExit("ffmpeg not on PATH")
    T = max(duration(args.zero), duration(args.ground))       # the GUI clip loops underneath
    pad = lambda p: f"tpad=stop_mode=clone:stop_duration={max(0.1, T - duration(p) + 0.1):.2f}"
    fc = (f"[0:v]scale=960:540,fps=20,{pad(args.zero)},{label('Microgravity 0 g (ISS)', 30, 470)}[a];"
          f"[1:v]scale=960:540,fps=20,{pad(args.ground)},"
          f"{label('Ground 9.81 m/s² - same module, planner, controller', 26, 470)}[b];"
          f"[2:v]scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2,fps=20,"
          f"{label('Isaac Sim GUI - PhysicsScene gravity 0 to 9.81 m/s² (scripted demo, live PhysX)', 30, 1020)}[c];"
          f"[a][b]hstack=inputs=2[top];[top][c]vstack=inputs=2,trim=duration={T:.2f}[v]")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(args.zero), "-i", str(args.ground), "-stream_loop", "-1", "-i", str(args.gui),
                    "-filter_complex", fc, "-map", "[v]", "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(args.out)], check=True)
    print(json.dumps({"video": str(args.out), "duration_s": round(duration(args.out), 1)}))


if __name__ == "__main__":
    main()

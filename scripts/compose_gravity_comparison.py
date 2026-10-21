"""Compose the microgravity vs ground comparison clip with ffmpeg (no simulation; inputs are rendered clips).

python scripts/compose_gravity_comparison.py [--zero outputs/iss_activity/fin_main_0g_v2.mp4]
    [--ground outputs/iss_activity/fin_main_1g.mp4] [--gui outputs/iss_activity/gui_gravity_raw.mp4]
Layout 1920x1080: top row = 0 g (left) | 9.81 m/s^2 (right), 960x540 each, same module, route, planner and
controller; bottom row = the Isaac Sim GUI gravity switch (864x540, looped) | a card explaining the comparison with
the numbers read from each run's <clip>_summary.json. A finished run holds its last frame with a "run stopped" note.
Output: outputs/iss_activity/iss_gravity_comparison.mp4.
"""
import argparse
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "iss_activity"
FONT = "C\\:/Windows/Fonts/malgun.ttf"          # Malgun Gothic: Hangul + Latin


def duration(p):
    r = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "json", str(p)],
                       capture_output=True, text=True, check=True)
    return float(json.loads(r.stdout)["format"]["duration"])


def ff_path(p):
    return str(p).replace("\\", "/").replace(":", "\\:")


def text(tf, size, x, y, extra=""):
    return (f"drawtext=fontfile='{FONT}':textfile='{ff_path(tf)}':fontsize={size}:fontcolor=white:line_spacing=10:"
            f"box=1:boxcolor=black@0.6:boxborderw=10:x={x}:y={y}{extra}")


def summary(clip):
    return json.loads((clip.parent / (clip.stem + "_summary.json")).read_text(encoding="utf-8"))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--zero", type=Path, default=OUT / "fin_main_0g_v2.mp4")
    ap.add_argument("--ground", type=Path, default=OUT / "fin_main_1g.mp4")
    ap.add_argument("--gui", type=Path, default=OUT / "gui_gravity_raw.mp4")
    ap.add_argument("--out", type=Path, default=OUT / "iss_gravity_comparison.mp4")
    args = ap.parse_args()
    if not shutil.which("ffmpeg"):
        raise SystemExit("ffmpeg not on PATH")
    sz, sg = summary(args.zero), summary(args.ground)
    dz, dg = duration(args.zero), duration(args.ground)
    T = max(dz, dg)
    tmp = OUT / "_compose_text"
    tmp.mkdir(exist_ok=True)

    def tf(name, s):
        p = tmp / f"{name}.txt"
        with open(p, "w", encoding="utf-8", newline="\n") as fh:      # CRLF renders as an extra line in drawtext
            fh.write(s)
        return p

    pr = sz.get("panel_reach") or {}
    zero_res = (f"{sz['completed_steps']}/{sz['total_steps']}걸음 포획"
                + (f", 패널 손 뻗기 {pr['min_distance_m'] * 100:.1f}cm" if pr else ""))
    g_err = (sg.get("capture_error_m") or [None])[-1]
    ground_res = (f"{sg['completed_steps']}/{sg['total_steps']}걸음 포획"
                  + (f" - {sg['completed_steps'] + 1}번째 손잡이를 {g_err * 100:.1f}cm 놓침(포획 반경 4cm)"
                     if g_err is not None and sg["completed_steps"] < sg["total_steps"] else ""))
    t_zero = tf("zero", "미세중력 0 g  (ISS 실제 조건)")
    t_ground = tf("ground", "지상 중력 9.81 m/s²  (같은 모듈을 지구에 둔 경우)")
    t_stop = tf("stop", f"실행 종료: {ground_res}")
    t_stopz = tf("stopz", f"실행 종료: {zero_res}")
    t_gui = tf("gui", "Isaac Sim GUI: PhysicsScene 중력 0 → 9.81 전환 (실제 PhysX)")
    t_card = tf("card", "\n".join([
        "무엇을 비교하나",
        "",
        "같은 ISS 모듈, 같은 G1 로봇, 같은 cuRobo 계획과 관절 제어기.",
        "바꾼 것은 PhysicsScene의 중력 하나뿐 (왼쪽 아래 Isaac Sim 화면).",
        "",
        f"왼쪽 0 g: {zero_res}.",
        f"오른쪽 9.81 m/s²: {ground_res}.",
        "",
        "플래너는 무중력을 가정하고 손잡이를 짚어 이동하므로,",
        "지구 중력에서는 몸이 처져 첫 손잡이부터 잡지 못한다.",
        "즉 이 이동 방식은 미세중력에서만 성립한다.",
        "",
        "2배속 재생 · 접촉 ON · 손잡이 파지는 고정 조인트(스크립트)",
    ]))
    hold_g = f"tpad=stop_mode=clone:stop_duration={max(0.1, T - dg + 0.1):.2f}"
    hold_z = f"tpad=stop_mode=clone:stop_duration={max(0.1, T - dz + 0.1):.2f}"
    fc = (f"[0:v]scale=960:540,fps=20,{hold_z},{text(t_zero, 30, '(w-text_w)/2', 'h-70')},"
          f"{text(t_stopz, 24, '(w-text_w)/2', 'h/2-20', f':enable=gte(t\\,{dz:.2f})')}[a];"
          f"[1:v]scale=960:540,fps=20,{hold_g},{text(t_ground, 30, '(w-text_w)/2', 'h-70')},"
          f"{text(t_stop, 24, '(w-text_w)/2', 'h/2-20', f':enable=gte(t\\,{dg:.2f})')}[b];"
          f"[2:v]scale=864:540:force_original_aspect_ratio=decrease,pad=864:540:(ow-iw)/2:(oh-ih)/2,fps=20,"
          f"{text(t_gui, 20, '(w-text_w)/2', 'h-44')}[c];"
          f"color=c=0x15181c:s=1056x540:r=20,{text(t_card, 22, 40, 36, ':box=0')}[d];"
          f"[a][b]hstack=inputs=2[top];[c][d]hstack=inputs=2[bot];[top][bot]vstack=inputs=2,trim=duration={T:.2f}[v]")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(args.zero), "-i", str(args.ground),
                    "-stream_loop", "-1", "-i", str(args.gui), "-filter_complex", fc, "-map", "[v]", "-c:v", "libx264",
                    "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.out)], check=True)
    shutil.rmtree(tmp, ignore_errors=True)
    print(json.dumps({"video": str(args.out), "duration_s": round(duration(args.out), 1)}))


if __name__ == "__main__":
    main()

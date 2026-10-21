"""Edit the ISS showcase reel from rendered clips with ffmpeg (no simulation; numbers come from the run summaries).

python scripts/compose_showcase.py [--out outputs/iss_demos/iss_showcase.mp4]
Sequence (1920x1080, 30 fps): title card -> main traverse (sped up) -> JEM | Node3 side by side -> Astrobee
tool handoff -> push-off momentum -> Earth from the Cupola -> 0 g vs ground comparison excerpt -> end card.
Every segment carries a caption naming what is simulated and what is scripted or placeholder; captions are
generated from the run/demo summary JSONs so the reel cannot drift from the records.
"""
import argparse
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ACT, DEM = ROOT / "outputs" / "iss_activity", ROOT / "outputs" / "iss_demos"
FONT = "C\\:/Windows/Fonts/malgun.ttf"
W, H, FPS = 1920, 1080, 30


def duration(p):
    r = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "json", str(p)],
                       capture_output=True, text=True, check=True)
    return float(json.loads(r.stdout)["format"]["duration"])


def summ(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def ffp(p):
    return str(p).replace("\\", "/").replace(":", "\\:")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--main", default="main_sv3")
    ap.add_argument("--jem", default="jem_sv3")
    ap.add_argument("--node3", default="node3_sv4")
    ap.add_argument("--out", type=Path, default=DEM / "iss_showcase.mp4")
    args = ap.parse_args()
    tmp = DEM / "_showcase"
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    sm, sj, sn = (summ(ACT / f"{t}_summary.json") for t in (args.main, args.jem, args.node3))
    sh, sp = summ(DEM / "handoff_summary.json"), summ(DEM / "pushoff_summary.json")

    def steps(s):
        return f"{s['completed_steps']}/{s['total_steps']}걸음"
    pr = sm.get("panel_reach") or {}
    mc = sp["momentum_conservation"]
    segs = []   # (name, filter input spec list, filter graph producing [v], caption, seconds)

    def card(name, text, secs):
        tf = tmp / f"{name}.txt"
        with open(tf, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
        segs.append((name, [], f"color=c=0x0d1117:s={W}x{H}:r={FPS}:d={secs},"
                     f"drawtext=fontfile='{FONT}':textfile='{ffp(tf)}':fontsize=44:fontcolor=white:line_spacing=18:"
                     f"x=(w-text_w)/2:y=(h-text_h)/2", secs))

    def clip(name, src, caption, speed=1.0, max_s=None):
        tf = tmp / f"{name}.txt"
        with open(tf, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(caption)
        d = duration(src) / speed
        secs = min(d, max_s) if max_s else d
        g = (f"setpts=PTS/{speed},fps={FPS},scale={W}:{H}:force_original_aspect_ratio=decrease,"
             f"pad={W}:{H}:(ow-iw)/2:(oh-ih)/2,trim=duration={secs:.2f},setpts=PTS-STARTPTS,"
             f"drawtext=fontfile='{FONT}':textfile='{ffp(tf)}':fontsize=34:fontcolor=white:box=1:"
             f"boxcolor=black@0.6:boxborderw=12:x=(w-text_w)/2:y=h-110")
        segs.append((name, [str(src)], g, secs))

    card("title", "ISS 안의 휴머노이드 로봇\nUnitree G1 · Isaac Sim 6.0.1 PhysX · 미세중력 0 g\n\n"
         "NASA ISS 내부 7개 모듈(USOS) · 손잡이 짚고 이동 · cuRobo 계획 + 폐루프 보정", 5)
    clip("main", ACT / f"{args.main}.mp4",
         f"Node1 → US Lab → Node2 → Columbus: {steps(sm)} 포획"
         + (f", 정비 패널 손 뻗기 {pr['min_distance_m'] * 100:.1f}cm" if pr else "") + " (6배속)", speed=3.0)
    # JEM | Node3 side by side
    tj, tn = tmp / "jem.txt", tmp / "node3.txt"
    for f, s, lbl in ((tj, sj, "Node2 → JEM"), (tn, sn, "Node1 → Node3")):
        with open(f, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(f"{lbl}: {steps(s)} 포획")
    dj, dn = duration(ACT / f"{args.jem}.mp4") / 2, duration(ACT / f"{args.node3}.mp4") / 2
    secs = max(dj, dn)
    g2 = (f"[{{a}}:v]setpts=PTS/2,fps={FPS},scale=960:540,tpad=stop_mode=clone:stop_duration={secs - dj + 0.1:.2f},"
          f"drawtext=fontfile='{FONT}':textfile='{ffp(tj)}':fontsize=30:fontcolor=white:box=1:boxcolor=black@0.6:"
          f"boxborderw=10:x=(w-text_w)/2:y=h-70[ja];"
          f"[{{b}}:v]setpts=PTS/2,fps={FPS},scale=960:540,tpad=stop_mode=clone:stop_duration={secs - dn + 0.1:.2f},"
          f"drawtext=fontfile='{FONT}':textfile='{ffp(tn)}':fontsize=30:fontcolor=white:box=1:boxcolor=black@0.6:"
          f"boxborderw=10:x=(w-text_w)/2:y=h-70[nb];[ja][nb]hstack=inputs=2,pad={W}:{H}:0:270,"
          f"trim=duration={secs:.2f},setpts=PTS-STARTPTS")
    segs.append(("jem_node3", [str(ACT / f"{args.jem}.mp4"), str(ACT / f"{args.node3}.mp4")], g2, secs))
    c = sh["catch"]
    clip("handoff", DEM / "handoff.mp4",
         f"Astrobee(NASA 모델, 9.09kg)가 공구 전달 → G1이 자유 팔로 추적해 잡음 ({c['status']}, 놓은 뒤 "
         f"{c['t_s'] - sh['release']['t_s']:.1f}초) · 추진은 힘 제어 근사, 잡기는 고정 조인트")
    clip("pushoff", DEM / "pushoff.mp4",
         f"밀고 놓기: 질량중심 {mc['com_speed_m_s']:.3f} m/s 표류 · 놓은 뒤 {mc['window_s'][1] - mc['window_s'][0]:.2f}초 동안 "
         f"운동량 변화 ≤ {mc['max_dP_kg_m_s']:.5f} kg·m/s (보존)", speed=1.5)
    clip("cupola", DEM / "cupola_earth.mp4",
         "Cupola 위치에서 본 지구 · 한 궤도 92.5분을 30초로 · NASA Blue Marble · 시각 연출(궤도 역학 없음)")
    comp = ACT / "iss_gravity_comparison.mp4"
    if comp.exists():
        clip("gravity", comp, "같은 모듈·같은 제어기, 중력만 0 → 9.81 m/s²: 지구 중력에선 첫 손잡이부터 실패", max_s=30.0)
    card("end", "정리\n\n"
         f"main {steps(sm)} · JEM {steps(sj)} · Node3 {steps(sn)} (접촉 ON)\n"
         "손잡이 일부는 공개 배치도가 없어 가상(PLACEHOLDER) · 파지는 스크립트 고정 조인트\n\n"
         "다음: 같은 실험 틀로 달 표면 1/6 g 보행 (Lunar v0)", 6)

    inputs, graphs, idx = [], [], 0
    for k, (name, srcs, g, secs) in enumerate(segs):
        if not srcs:
            graphs.append(f"{g},format=yuv420p[s{k}]")
            continue
        if name == "jem_node3":
            a, b = idx, idx + 1
            graphs.append(g.replace("{a}", str(a)).replace("{b}", str(b)) + f",format=yuv420p[s{k}]")
        else:
            graphs.append(f"[{idx}:v]{g},format=yuv420p[s{k}]")
        inputs += srcs
        idx += len(srcs)
    fc = ";".join(graphs) + ";" + "".join(f"[s{k}]" for k in range(len(segs))) + f"concat=n={len(segs)}:v=1:a=0[v]"
    cmd = ["ffmpeg", "-y", "-loglevel", "error"]
    for s in inputs:
        cmd += ["-i", s]
    cmd += ["-filter_complex", fc, "-map", "[v]", "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", str(args.out)]
    subprocess.run(cmd, check=True)
    shutil.rmtree(tmp, ignore_errors=True)
    print(json.dumps({"video": str(args.out), "duration_s": round(duration(args.out), 1),
                      "segments": [(n, round(s, 1)) for n, _, _, s in segs]}))


if __name__ == "__main__":
    main()

"""Edit the ISS -> Moon showcase v2 from clean renders (docs/SHOWCASE_STORYBOARD.md); no simulation.

python scripts/compose_showcase_v2.py [--out outputs/iss_demos/iss_showcase_v2.mp4]
Inputs: overlay-free clips and their per-frame sidecars in outputs/showcase_v2/ (render_iss_activity.py --clean,
render_cupola_earth.py --clean, render_lunar_footprints.py --pov). All on-screen text is drawn here from one style
table; every number is formatted from a run/result JSON field (or a sidecar written from the run record) at build
time, and every string is checked (no Hangul, no banned phrases, no ! ? or em dash, <= 60 characters per line)
before ffmpeg runs. Acts are joined with a 12-frame cross-dissolve; shots inside an act are hard cuts.
"""
import argparse
import json
import re
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageFont

ROOT = Path(__file__).resolve().parents[1]
ACT, DEM, LUN, V2 = (ROOT / "outputs" / d for d in ("iss_activity", "iss_demos", "lunar_v0", "showcase_v2"))
W, H, FPS, M = 1920, 1080, 30, 96                      # frame, safe margin
F_SEMI, F_REG = "C:/Windows/Fonts/seguisb.ttf", "C:/Windows/Fonts/segoeui.ttf"   # Segoe UI 600 / 400 (no Inter)
SIZE = {"title": 64, "label": 34, "caption": 28, "readout": 26, "credits": 22, "badge": 26}
WHITE, ACCENT = "0xF2F2F2", "0xFFB000"
FADE = 8 / FPS
XF = 12 / FPS                                         # act dissolve
BANNED = ["welcome", "let's", "imagine", "journey", "showcasing", "cutting-edge", "seamless", "powered by", "here:",
          "next stage"]
TEXTS = []                                            # every overlay string, for the check
# which runs a cut is built from: v2 = the 2026-10-25 cut (self-collision off, limbs could pass through the body);
# v3 = the same storyboard from the self-collision runs (docs/ISS_TO_LUNAR_SHOWCASE_ARCHITECTURE.md, sc4)
CUTS = {
    "v2": dict(clips="showcase_v2", main="main_sv5", jem="jem_sv5", node3="node3_sv5", zero="fin_main_0g_v2",
               one="fin_main_1g", handoff="handoff", pushoff="pushoff", moon="p3_moon_scratch_at_moon_eval",
               receipt="All three routes with one code version", self_collision=False),
    "v3": dict(clips="showcase_v3", main="sc4_main", jem="sc4_jem", node3="sc4_node3", zero="sc4_main",
               one="sc4_main_1g", handoff="sc4_handoff", pushoff="sc4_pushoff", moon="p7_1300_at_moon_eval",
               receipt="Self-collision on", self_collision=True),
    # v4: calmer motion (planner pelvis weights, slower moves, more damping) at 3x; the main route is shown in three
    # parts (start, panel reach, finish) because 760 s at 3x would not fit
    "v4": dict(clips="showcase_v4", main="calm2_main", jem="slow2_jem", node3="slow2_node3", zero="calm2_main",
               one="calm2_main_1g", handoff="calm2_handoff", pushoff="calm2_pushoff", moon="p7_1300_at_moon_eval",
               receipt="Calmer motion", self_collision=True, calm=True,
               main_parts=("c03a_main", "c03b_main", "c03c_main")),
    # v5: same runs as v4, routes as one continuous speed ramp each (scripts/speed_map.py: fast in transit, slow at
    # selected captures and the panel reach; the badge follows the speed), Earth seen through the Cupola windows
    "v5": dict(clips="showcase_v5", main="calm2_main", jem="slow2_jem", node3="slow2_node3", zero="calm2_main",
               one="calm2_main_1g", handoff="calm2_handoff", pushoff="calm2_pushoff", moon="p7_1300_at_moon_eval",
               receipt="Calmer motion", self_collision=True, calm=True, window=True, teaser=True,
               title="Unitree G1, from the ISS to the Moon"),
}


def js(p):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def ffp(p):
    return str(p).replace("\\", "/").replace(":", "\\:")


def textfile(t):
    """Each overlay string goes through a file, so user text needs no filtergraph escaping."""
    d = V2 / "_txt"
    d.mkdir(exist_ok=True)
    f = d / f"t{len(list(d.glob('t*.txt'))):04d}.txt"
    f.write_text(t, encoding="utf-8", newline="\n")
    return ffp(f)


def width(text, kind):
    return ImageFont.truetype(F_SEMI if kind in ("title", "label") else F_REG, SIZE[kind]).getlength(text)


def alpha(a, b):
    """8-frame fade in at a, fade out ending at b (shot-local seconds)."""
    return (f"if(lt(t\\,{a:.3f})\\,0\\,if(lt(t\\,{a + FADE:.3f})\\,(t-{a:.3f})/{FADE:.3f}\\,"
            f"if(lt(t\\,{b - FADE:.3f})\\,1\\,if(lt(t\\,{b:.3f})\\,({b:.3f}-t)/{FADE:.3f}\\,0))))")


def text(t, kind, x, y, a, b, color=WHITE, box=False, fade=True, en=None):
    """drawtext for one string shown over [a, b) of the shot; x/y are ffmpeg expressions or numbers."""
    for line in t.split("\n"):
        TEXTS.append(line)
    font = F_SEMI if kind in ("title", "label") else F_REG
    f = (f"drawtext=fontfile='{ffp(font)}':textfile='{textfile(t)}':expansion=none:fontsize={SIZE[kind]}:"
         f"fontcolor={color}:line_spacing={int(SIZE[kind] * 0.45)}:x={x}:y={y}")
    if box:
        f += ":box=1:boxcolor=black@0.4:boxborderw=12"
    f += f":alpha={alpha(a, b)}" if fade else ""
    return f + f":enable=between(t\\,{(en or (a, b))[0]:.3f}\\,{(en or (a, b))[1]:.3f})"


def label(t, a, b):                                    # scene label, top left
    return text(t, "label", M, M, a, b, box=True)


def caption(t, a, b):                                  # caption, lower left (max two lines)
    return text(t, "caption", M, f"h-{M}-text_h", a, b, box=True)


def badge(t, a, b):                                    # speed badge, top right
    return text(t, "badge", f"w-{M}-text_w", M, a, b, box=True)


def speed_badges(sidecar, d):
    """Speed badge(s) for a clip: one per constant-speed run when the render used a speed map, else the clip speed."""
    j = js(sidecar)
    fr = j["frames"]
    if not fr or "speed" not in fr[0]:
        return [badge(f"{j['speed']:g}×", 0.0, d)] if j["speed"] != 1 else []
    out, a = [], 0
    for k in range(1, len(fr) + 1):
        if k == len(fr) or fr[k]["speed"] != fr[a]["speed"]:
            b = k / FPS if k < len(fr) else d
            out.append(text(f"{fr[a]['speed']:g}×", "badge", f"w-{M}-text_w", M, a / FPS, b, box=True,
                            fade=False))
            a = k
    return out


def readout(name, values, a, b, y_off=0):
    """Data readout, lower right: white name + accent number. values: [(t0, t1, number text)] in shot seconds;
    the block fades in/out over [a, b) as a whole, the number switches with hard steps."""
    nw = max(width(v, "readout") for _, _, v in values)
    y = f"h-{M}-{SIZE['readout'] + y_off}"
    fs = [text(name, "readout", f"w-{M}-{nw:.0f}-16-text_w", y, a, b, box=True)]
    for t0, t1, v in values:
        fs.append(text(v, "readout", f"w-{M}-{nw:.0f}", y, a, b, color=ACCENT, box=True,
                       en=(max(t0, a), min(t1, b))))
    return fs


def steps(sidecar, total, dur, final=None):
    """[(t0, t1, 'N / total')] from a render sidecar's per-frame capture count; final (the run summary's count)
    replaces the last value when the record ends before the last capture is logged."""
    fr = js(sidecar)["frames"]
    out, cur, t0 = [], None, 0.0
    for k, f in enumerate(fr):
        v = f["captures"]
        if v != cur:
            if cur is not None:
                out.append((t0, k / FPS, f"{cur} / {total}"))
            cur, t0 = v, k / FPS
    if final is not None and final > cur:
        out.append((t0, max(t0, dur - 5.0), f"{cur} / {total}"))
        cur, t0 = final, max(t0, dur - 5.0)
    out.append((t0, dur, f"{cur} / {total}"))
    return out


def frames(p):
    return len(js(p)["frames"])


def exposure_gain(stem):
    """Mean luma of the clip's first and last 2 s of frames (from the clean render's PNGs)."""
    fd = next(d for d in (ACT, DEM) if (d / f"render_frames_{stem}").exists()) / f"render_frames_{stem}"
    pngs = sorted(fd.glob("f*.png"))
    pick = pngs[:2 * FPS:10] + pngs[-2 * FPS::10]
    return float(np.mean([np.asarray(Image.open(p).convert("L").resize((192, 108)), float).mean() for p in pick]))


def gate_text(ev):
    """Caption naming the failed velocity gates of a lunar evaluation, measured value and limit."""
    g = ev["gates"]
    bad = []
    if ev["tracking_rms_m_s"] >= g["tracking_rms_max"]:
        bad.append(f"tracking {ev['tracking_rms_m_s']:.2f} m/s (limit {g['tracking_rms_max']:g})")
    if ev["slip_median_m_s"] >= g["slip_median_max"]:
        bad.append(f"slip {ev['slip_median_m_s']:.3f} m/s (limit {g['slip_median_max']:g})")
    return "Walking gates not met\n" + ", ".join(bad) if bad else "Walking gates met"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cut", default="v2", choices=sorted(CUTS))
    ap.add_argument("--out", type=Path, help="default: outputs/iss_demos/iss_showcase_<cut>.mp4")
    ap.add_argument("--preview", default="", help="comma-separated shot names: hard-cut preview of those shots only")
    args = ap.parse_args()
    cut = CUTS[args.cut]
    global V2
    V2 = ROOT / "outputs" / cut["clips"]
    args.out = args.out or DEM / f"iss_showcase_{args.cut}.mp4"
    want = set(args.preview.split(",")) if args.preview else {
        "open", "title", "main", "jem", "node3", "handoff", "pushoff", "cupola", "gravity", "moonwalk", "moon", "black",
        "final", "credits"}
    for f in (V2 / "_txt").glob("t*.txt"):
        f.unlink()

    sm, sj, sn = (js(ACT / f"{cut[r]}_summary.json") for r in ("main", "jem", "node3"))
    sh, sp = js(DEM / f"{cut['handoff']}_summary.json"), js(DEM / f"{cut['pushoff']}_summary.json")
    s0g, s1g = js(ACT / f"{cut['zero']}_summary.json"), js(ACT / f"{cut['one']}_summary.json")
    g0 = js(ACT / f"{cut['zero']}_run.json")["gravity_m_s2"]
    g1 = js(ACT / f"{cut['one']}_run.json")["gravity_m_s2"]
    ev = js(LUN / f"{cut['moon']}.json")["summary"]
    isaac = Path("C:/isaacsim/VERSION").read_text(encoding="utf-8").split("-")[0].strip()
    commit = re.search(re.escape(cut["receipt"]) + r" \([^)]*commit ([0-9a-f]{7})\)",
                       (ROOT / "docs" / "ISS_TO_LUNAR_SHOWCASE_ARCHITECTURE.md").read_text(encoding="utf-8")).group(1)
    g_moon = f"{abs(ev['gravity_m_s2']):.2f}"
    mc = sp["momentum_conservation"]

    # exposure matching across the ISS interior shots only (Cupola and Moon are mostly black sky by design)
    iss = ["c01_open", *(cut.get("main_parts") or ["c03_main"]), "c04_jem", "c05_node3", "c06_handoff", "c07_pushoff",
           "c09_zero", "c09_one"]
    lum = {s: exposure_gain(s) for s in iss if (V2 / f"{s}.mp4").exists()}
    ref = float(np.median(list(lum.values())))
    gain = {s: float(np.clip(ref / lum[s], 0.8, 1.6)) for s in lum}
    if args.preview == "" and len(lum) < len(iss):
        raise SystemExit(f"missing renders: {sorted(set(iss) - set(lum))}")

    def src(stem, extra=""):
        g = gain.get(stem, 1.0)
        lut = f"lutrgb=r='clipval*{g:.3f}':g='clipval*{g:.3f}':b='clipval*{g:.3f}'," if abs(g - 1) > 0.02 else ""
        return f"fps={FPS},scale={W}:{H},{lut}{extra}setsar=1"

    shots = []        # (name, [input files], filter body using {i0},{i1}, duration)

    def add(name, inputs, body, dur):
        shots.append((name, inputs, body, dur))

    def stopped(s, d):
        """Caption + readout for a route that stopped before its last handhold (last 5 s of the shot)."""
        if s["completed_steps"] >= s["total_steps"]:
            return []
        miss = s["capture_error_m"][-1] * 100
        return [caption(f"Stopped at handhold {s['completed_steps'] + 1} of {s['total_steps']}", d - 5.0, d),
                *readout("Hand to handhold", [(d - 5.0, d, f"{miss:.1f} cm")], d - 5.0, d, y_off=SIZE["readout"] + 30)]

    def clip(name, stem, dur, overlays, extra=""):
        add(name, [V2 / f"{stem}.mp4"], f"[{{i0}}:v]{src(stem, extra)},trim=duration={dur:.3f},setpts=PTS-STARTPTS,"
            + ",".join(overlays), dur)

    # 1 cold open; v5: a 10 s teaser of the film's strongest moments instead (owner: an intro that holds viewers),
    # cut from the same clips, each with its playback speed, then the title
    if "open" in want and cut.get("teaser"):
        def at_speed(stem, ss):
            j = js(V2 / f"{stem}_frames.json")
            fr = j["frames"]
            k = min(int(ss * FPS), len(fr) - 1)
            return fr[k].get("speed", j.get("speed", 1.0)) if fr else j.get("speed", 1.0)

        def first_slow(stem):
            fr = js(V2 / f"{stem}_frames.json")["frames"]
            k = next(i for i, f in enumerate(fr) if f.get("speed", 99) < 2)
            return k / FPS + 0.6

        dpov = frames(V2 / "c10_pov_frames.json") / FPS
        parts = [("c08_cupola", 0.2, 2.2), ("c03_main", first_slow("c03_main"), 2.0), ("c07_pushoff", 1.5, 1.8),
                 ("c09_one", 10.5, 1.8), ("c10_walk", 1.0, 1.6), ("c10_pov", dpov - 2.2, 2.2)]
        for n, (stem, ss, dur) in enumerate(parts):
            spd = at_speed(stem, ss)
            add(f"teaser{n}", [V2 / f"{stem}.mp4"],
                f"[{{i0}}:v]{src(stem)},trim=start={ss:.3f}:duration={dur:.3f},setpts=PTS-STARTPTS"
                + ("," + text(f"{spd:g}×" if float(spd).is_integer() or spd < 10 else f"≈ {spd:.0f}×", "badge",
                              f"w-{M}-text_w", M, 0.0, dur, box=True, fade=False)
                   if spd != 1 else ""), dur)
    elif "open" in want:
        d = frames(V2 / "c01_open_frames.json") / FPS
        clip("open", "c01_open", d, [label("ISS · US Laboratory", 4.0, d)])
    # 2 title
    if "title" in want:
        d = 6.0
        t1 = cut.get("title", "Unitree G1 aboard the ISS")
        t2 = f"Simulated in NVIDIA Isaac Sim {isaac} with PhysX" + ("" if cut.get("teaser") else ", 0 g")
        add("title", [], f"color=c=black:s={W}x{H}:r={FPS}:d={d}," + ",".join([
            text(t1, "title", "(w-text_w)/2", "h/2-80", 0.3, d - 0.3),
            text(t2, "caption", "(w-text_w)/2", "h/2+20", 0.6, d - 0.3)]), d)
    # 3 main route, full, 8x; 10 % eased push-in over the last 6 s
    if "main" in want and cut.get("main_parts"):
        reach = sm["panel_reach"]["min_distance_m"] * 100
        lbl = "Node 1 → US Lab → Node 2 → Columbus"
        for k, stem in enumerate(cut["main_parts"]):
            sc = V2 / f"{stem}_frames.json"
            d = frames(sc) / FPS
            ov = [label(lbl, 0.0, d), badge(f"{js(sc)['speed']:g}×", 0.0, d),
                  *readout("Handholds", steps(sc, sm["total_steps"], d,
                                             sm["completed_steps"] if k == len(cut["main_parts"]) - 1 else None),
                           0.0, d)]
            tp = next((i for i, f in enumerate(js(sc)["frames"]) if f["phase"] == "panel_reach"), None)
            if k == 0:
                ov.append(caption("Hand over hand, contacts and self-collision on", 4.0, d))
            if tp is not None:
                tp /= FPS
                ov += [caption("Reach to the service panel", tp, tp + 4.0),
                       *readout("Panel reach", [(tp, tp + 4.0, f"{reach:.1f} cm")], tp, tp + 4.0,
                                y_off=SIZE["readout"] + 30)]
            if k == len(cut["main_parts"]) - 1:
                ov += stopped(sm, d) or [caption(f"{sm['completed_steps']} of {sm['total_steps']} handholds",
                                                 max(0.0, d - 5.0), d)]
            clip(f"main{'abc'[k]}", stem, d, ov)
    elif "main" in want:
        sc = V2 / "c03_main_frames.json"
        sp3 = js(sc)["speed"]
        d = frames(sc) / FPS
        z = f"(1+0.10*if(lt(t\\,{d - 6:.3f})\\,0\\,(1-cos(PI*min(1\\,(t-{d - 6:.3f})/6)))/2))"
        push = f"scale=w=ceil({W}*{z}/2)*2:h=ceil({H}*{z}/2)*2:eval=frame,crop={W}:{H},"
        reach = sm["panel_reach"]["min_distance_m"] * 100
        tp = next(k for k, f in enumerate(js(sc)["frames"]) if f["phase"] == "panel_reach") / FPS
        c1 = ("Hand over hand, contacts and self-collision on" if cut["self_collision"] else
              "Hand over hand along the handholds, contacts on")
        clip("main", "c03_main", d, [label("Node 1 → US Lab → Node 2 → Columbus", 0.0, d), *speed_badges(sc, d),
                                     caption(c1, 6.0, tp - 0.3),
                                     caption("Reach to the service panel", tp, tp + 4.0),
                                     *readout("Handholds", steps(sc, sm["total_steps"], d), 0.0, d),
                                     *readout("Panel reach", [(tp, tp + 4.0, f"{reach:.1f} cm")], tp, tp + 4.0,
                                              y_off=SIZE["readout"] + 30),
                                     *stopped(sm, d)], extra=push)
    # 4 / 5 JEM, Node 3 (last 120 s of each route, 8x)
    if {"jem", "node3"} & want:
        for name, stem, lbl, cap, s in (("jem", "c04_jem", "Node 2 → JEM", "Same controller, second route", sj),
                                        ("node3", "c05_node3", "Node 1 → Node 3",
                                         f"Three routes, commit {commit}, one run each", sn)):
            sc = V2 / f"{stem}_frames.json"
            d = frames(sc) / FPS
            end = d - 5.0 if s["completed_steps"] < s["total_steps"] else d
            clip(name, stem, d, [label(lbl, 0.0, d), *speed_badges(sc, d), caption(cap, 1.0, end - 0.3),
                                 *readout("Handholds", steps(sc, s["total_steps"], d), 0.0, d), *stopped(s, d)])
    # 6 hand-off, 1x
    if "handoff" in want:
        sc = V2 / "c06_handoff_frames.json"
        d0 = frames(sc) / FPS
        t_catch = next(k for k, f in enumerate(js(sc)["frames"]) if f["t"] >= sh["catch"]["t_s"]) / FPS
        hold = max(0.0, t_catch + 3.0 - d0)            # freeze the last frame so the catch readout holds >= 2.5 s
        d = d0 + hold
        spd = js(sc)["speed"]
        clip("handoff", "c06_handoff", d, [label("Tool hand-off", 0.0, d),
                                           *([badge(f"{spd:g}×", 0.0, d0)] if spd != 1 else []),
                                           caption("Astrobee releases a tool bar. G1 catches it.", 1.0, d),
                                           *readout("Release to catch",
                                                    [(t_catch, d, f"{sh['catch']['t_s'] - sh['release']['t_s']:.1f} s")],
                                                    t_catch, d)],
             extra=f"tpad=stop_mode=clone:stop_duration={hold + 0.1:.2f}," if hold else "")
    # 7 push-off, 2x
    if "pushoff" in want:
        sc = V2 / "c07_pushoff_frames.json"
        d = frames(sc) / FPS
        t_rel = next(k for k, f in enumerate(js(sc)["frames"]) if f["t"] >= sp["release_t_s"]) / FPS
        t_cap2 = max(t_rel, 3.0)
        win = mc["window_s"][1] - mc["window_s"][0]
        clip("pushoff", "c07_pushoff", d, [label("Push-off", 0.0, d), badge(f"{js(sc)['speed']:g}×", 0.0, d),
                                           caption("G1 pushes off and lets go", 0.5, t_cap2),
                                           caption("Momentum constant from release to first contact", t_cap2, d),
                                           *readout(f"Δp over {win:.2f} s", [(t_rel, d, f"≤ {mc['max_dP_kg_m_s']:g} kg·m/s")],
                                                    t_rel, d),
                                           *readout("CoM drift", [(t_rel, d, f"{mc['com_speed_m_s']:.3f} m/s")],
                                                    t_rel, d, y_off=SIZE["readout"] + 30)])
    # 8 Cupola Earth (day pass into sunset)
    if "cupola" in want:
        sc = V2 / "c08_cupola_frames.json"
        cj = js(sc)
        d = frames(sc) / FPS
        mins, vals, cur, t0 = [int(f["orbit_min"]) for f in cj["frames"]], [], None, 0.0
        for k, v in enumerate(mins + [None]):
            if v != cur:
                if cur is not None:
                    vals.append((t0, k / FPS, f"{cur} min"))
                cur, t0 = v, k / FPS
        clip("cupola", "c08_cupola", d, [label("Earth through the Cupola windows" if cut.get("window") else
                                               "Earth from the Cupola position", 0.0, d),
                                         badge(f"≈ {cj['speed']:.0f}×", 0.0, d),
                                         *readout("Orbit time", vals, 0.5, d)])
    # 9 0 g | 9.81 m/s^2, full height side by side, 4x, then a 3 s hold on the last frames
    if "gravity" in want:
        sz, so = V2 / "c09_zero_frames.json", V2 / "c09_one_frames.json"
        dz, do = frames(sz) / FPS, frames(so) / FPS
        run = max(dz, do)
        hold = 3.0
        d = run + hold
        half = f"crop={W // 2}:{H}:{W // 4}:0"
        lbl_r = f"{g1:g} m/s²"
        left = (f"[{{i0}}:v]{src('c09_zero')},{half},tpad=stop_mode=clone:stop_duration={d - dz + 0.1:.2f},"
                f"trim=duration={d:.3f},setpts=PTS-STARTPTS,"
                + ",".join([text(f"{g0:g} g", "label", M, M, 0.0, d, box=True),
                            # one row up: the full-width caption sits on the left half's bottom line
                        *readout("Handholds", steps(sz, s0g["total_steps"], d), 0.0, d,
                                 y_off=SIZE["readout"] + 30)]) + "[L]")
        miss = s1g["capture_error_m"][-1] * 100
        right = (f"[{{i1}}:v]{src('c09_one')},{half},tpad=stop_mode=clone:stop_duration={d - do + 0.1:.2f},"
                 f"trim=duration={d:.3f},setpts=PTS-STARTPTS,"
                 + ",".join([text(lbl_r, "label", M, M, 0.0, d, box=True),
                             badge(f"{js(so)['speed']:g}×", 0.0, run),
                             *readout("Handholds", steps(so, s1g["total_steps"], d), 0.0, run),
                             *readout(f"Miss at handhold {s1g['completed_steps'] + 1}", [(do, d, f"{miss:.1f} cm")],
                                      do, d)]) + "[R]")
        add("gravity", [V2 / "c09_zero.mp4", V2 / "c09_one.mp4"],
            left + ";" + right + ";[L][R]hstack=inputs=2,"
            + caption("Same route and controller. Only gravity changes.", 1.0, d), d)
    # 10a lunar side-on walk (v3: shows the longer stride), 1x; 10 lunar POV, 0.5x
    walk = "moonwalk" in want and (V2 / "c10_walk.mp4").exists()
    if walk:
        sc = V2 / "c10_walk_frames.json"
        d = frames(sc) / FPS
        add("moonwalk", [V2 / "c10_walk.mp4"], f"[{{i0}}:v]fps={FPS},scale={W}:{H},setsar=1,trim=duration={d:.3f},"
            "setpts=PTS-STARTPTS," + ",".join([
                label("Moon · learned walking policy", 0.3, d), caption(gate_text(ev), 1.0, d - 0.3),
                *readout("g", [(0.0, d, f"{g_moon} m/s²")], 0.3, d)]), d)
    if "moon" in want:
        sc = V2 / "c10_pov_frames.json"
        pj = js(sc)
        d = frames(sc) / FPS
        add("moon", [V2 / "c10_pov.mp4"], f"[{{i0}}:v]fps={FPS},scale={W}:{H},setsar=1,trim=duration={d:.3f},"
            "setpts=PTS-STARTPTS," + ",".join([
                badge(f"{pj['speed']:g}×", 0.0, d), *readout("g", [(0.0, d, f"{g_moon} m/s²")], 0.5, d),
                *([] if walk else [label("Moon · learned walking policy", 0.5, d),
                                   caption(gate_text(ev), 1.0, d - 0.5)])])
            + f",fade=t=out:st={d - FADE:.3f}:d={FADE:.3f}", d)
    # 11-13 black, final card, credits
    if {"black", "final", "credits"} & want:
        add("black", [], f"color=c=black:s={W}x{H}:r={FPS}:d=1", 1.0)
        d = 6.0
        add("final", [], f"color=c=black:s={W}x{H}:r={FPS}:d={d}," + ",".join([
            text(f"{g_moon} m/s². No atmosphere.", "title", "(w-text_w)/2", "h/2-70", 0.3, d - 0.3),
            text("Unitree G1 in NVIDIA Isaac Sim", "label", "(w-text_w)/2", "h/2+20", 0.6, d - 0.3)]), d)
        credits = [f"ISS routes: commit {commit}, one run each, contacts on.",
                   "Repeatability across runs was not measured.",
                   "Grasps are scripted constraints (fixed joints).",
                   "Some handholds and the panel anchor are placeholders.",
                   *([f"Hand-off, push-off and the {g1:g} m/s² comparison start",
                      "from an earlier controller run."] if not cut["self_collision"] else
                     (["Main route: the planner holds the body orientation.",
                       "JEM, Node 3: slower moves and more damping only."] if cut.get("calm") else []) +
                     ["Self-collision is on in planning and physics.",
                      "Hand, pelvis and waist colliders come from visual meshes.",
                      "Hand-off and push-off start at main-route handhold 9."]),
                   "Astrobee propulsion is a force-limited stand-in.",
                   ("Earth is rendered; the Cupola window panes are removed." if cut.get("window") else
                    "Earth view is rendered; the Cupola mesh is hidden."),
                   "Footprints and dust launch are rendered effects.",
                   f"Dust follows ballistic paths at {g_moon} m/s².",
                   "Moon: Isaac Lab G1 model, not the ISS G1 model.",
                   "ISS interior: NASA 3D Resources via nasa/astrobee_media.",
                   "Earth: NASA Blue Marble Next Generation.",
                   f"Simulation: NVIDIA Isaac Sim {isaac}, Isaac Lab, cuRobo."]
        d = 14.0
        cw = max(width(c, "credits") for c in credits)
        lh = int(SIZE["credits"] * 1.6)
        y0 = (H - lh * len(credits)) // 2
        add("credits", [], f"color=c=black:s={W}x{H}:r={FPS}:d={d}," + ",".join(
            text(c, "credits", f"{(W - cw) / 2:.0f}", y0 + k * lh, 0.3, d - 0.3) for k, c in enumerate(credits)), d)

    # text check before anything is encoded
    bad = []
    for t in TEXTS:
        low = t.lower()
        if re.search(r"[\u1100-\u11ff\u3130-\u318f\uac00-\ud7af]", t):
            bad.append(("hangul", t))
        if any(b in low for b in BANNED) or re.search(r"[!?—]", t) or len(t) > 60:
            bad.append(("style", t))
    if bad:
        raise SystemExit(f"text check failed: {bad}")

    # filter graph: shots -> acts (hard cuts) -> acts joined by 12-frame dissolves
    inputs, graphs, idx = [], [], 0
    for k, (name, ins, body, dur) in enumerate(shots):
        b = body
        for j in range(len(ins)):
            b = b.replace(f"{{i{j}}}", str(idx + j))
        graphs.append(f"{b},format=yuv420p,setpts=PTS-STARTPTS[s{k}]" if ins else f"{b},format=yuv420p[s{k}]")
        inputs += [str(p) for p in ins]
        idx += len(ins)
    names = [s[0] for s in shots]
    if args.preview:                                   # hard-cut preview of the requested shots
        acts = [names]
    else:
        acts = [names[:names.index("node3") + 1], names[names.index("handoff"):names.index("gravity") + 1],
                names[names.index("moonwalk" if "moonwalk" in names else "moon"):]]
    act_d = []
    for a, members in enumerate(acts):
        ks = [names.index(m) for m in members]
        graphs.append("".join(f"[s{k}]" for k in ks) + f"concat=n={len(ks)}:v=1:a=0[a{a}]")
        act_d.append(sum(shots[k][3] for k in ks))
    if args.preview:
        graphs[-1] = graphs[-1].replace("[a0]", "[v]")
    else:
        off1 = act_d[0] - XF
        off2 = off1 + act_d[1] - XF
        graphs.append(f"[a0][a1]xfade=transition=fade:duration={XF:.3f}:offset={off1:.3f}[x1]")
        graphs.append(f"[x1][a2]xfade=transition=fade:duration={XF:.3f}:offset={off2:.3f}[v]")
    cmd = ["ffmpeg", "-y", "-loglevel", "error"]
    for s in inputs:
        cmd += ["-i", s]
    fc = V2 / "_filter.txt"
    fc.write_text(";\n".join(graphs), encoding="utf-8")
    cmd += ["-filter_complex_script", str(fc), "-map", "[v]", "-r", str(FPS), "-c:v", "libx264", "-crf", "18",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.out)]
    subprocess.run(cmd, check=True)
    t = 0.0
    bounds = []
    for a, members in enumerate(acts):
        for m in members:
            bounds.append((m, round(t, 2)))
            t += shots[names.index(m)][3]
        t -= XF if a < len(acts) - 1 else 0.0
    (V2 / "v2_cut.json").write_text(json.dumps({"video": str(args.out), "shots": bounds, "end_s": round(t, 2),
                                                "gain": gain, "texts": sorted(set(TEXTS))}, ensure_ascii=False,
                                               indent=1), encoding="utf-8")
    print(json.dumps({"video": str(args.out), "end_s": round(t, 2), "shots": bounds}, ensure_ascii=False))


if __name__ == "__main__":
    main()

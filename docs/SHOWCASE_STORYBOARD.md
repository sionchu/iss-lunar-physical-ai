# ISS → Moon showcase v2: storyboard (G2)

Status: draft for approval (2026-10-25). Nothing below is rendered yet. Brief: owner's `showcase_storyboard_prompt.md`.
Output after approval: `outputs/iss_demos/iss_showcase_v2.mp4` (1920×1080, 30 fps), composed by `scripts/compose_showcase_v2.py`.
The current cut `outputs/iss_demos/iss_showcase.mp4` is kept unchanged.

## Decisions taken at G1 (owner: "추천 진행")

- Font: Segoe UI (Inter is not installed). `seguisb.ttf` (Semibold, 600) for titles and labels, `segoeui.ttf` (Regular, 400) for body text.
- Lunar POV pitch: about 15° down while walking, so the horizon and Earth are in frame. On the last step it eases to 35° down over ≥ 3 s. A fixed 35° pitch with a 70° horizontal FOV (≈ 43° vertical at 16:9) puts the top edge ≈ 13° below the horizon, so Earth cannot be in frame.

## Production rules for this cut

- **Clean sources, no burned-in overlays.**
  - Every ISS shot is re-rendered from its existing run record with a new `--clean` flag in `scripts/render_iss_activity.py`. The flag skips the PIL overlay and writes a per-frame sidecar JSON with `t`, current step and captures so far.
  - Nothing is re-simulated; the renderer replays recorded link poses.
  - Renders are 1920×1080. For each shot, `--every` and `--speed` are chosen so the output is 30 fps at the speed on the badge. Only the board's windows are rendered (`--start-s`/`--end-s`).
- **Text layer.** All text is drawn by the compose script with ffmpeg `drawtext`, from one style table:
  - Positions (96 px safe margin): scene label top left; caption lower left; data readout lower right; speed badge top right.
  - Sizes: title 64, scene label 34, caption 28, readout 26, credits 22 px.
  - Colour: text #F2F2F2. Live numbers use #FFB000.
  - Backing bar: 40 % black, only behind captions and readouts on busy footage.
  - Fades: 8-frame fade in and out; each text holds ≥ 2.5 s.
- **Numbers.** Every number on screen is formatted from a JSON field at compose time (table below); the script holds no numeric literals for on-screen text. The script fails rather than render a missing field.
- **Text check.** Before encoding, the script checks every overlay string for:
  - Hangul code points;
  - the banned phrases from the brief;
  - `!`, `?` and em dashes;
  - lines over 60 characters.
  A match aborts the build.
- **Transitions.** Hard cuts inside an act. A 12-frame cross-dissolve only at the act boundaries ISS → demos and demos → Moon, plus the fade to black before the final card.
- **Grade.** Exposure matching only: a per-shot gain from the mean luma of the shot's first and last 2 s, toward the median of the ISS shots. No colour grading.
- **Audio.** None.

## Shot list

Times are planned; the compose script derives the exact cut points from the clip lengths, and G4 checks the result against this table.
Speed = screen time ÷ simulated time.

| # | time in–out | source clip / rollout | framing & camera move | on-screen text (exact English) | data readout & source field | effect / transition | speed |
|---|---|---|---|---|---|---|---|
| 1 | 0:00–0:08 | `main_sv5_run.json`, sim 100–108 s (US Lab, mid-traverse) | existing chase camera on the module centre line (recorded path, smoothed) | none for 0–4 s; from 4 s, label: `ISS · US Laboratory` | none | cold open from black, hard cut in | 1× (no badge) |
| 2 | 0:08–0:14 | black card | static | title (64 px, 600): `Unitree G1 aboard the ISS` / body (28 px): `Simulated in NVIDIA Isaac Sim 6.0.1 with PhysX, 0 g` | none (version string from the brief's environment, not a measurement) | hard cut | n/a |
| 3 | 0:14–0:58 | `main_sv5_run.json`, full route, sim 0–349.5 s | chase camera, eased 10 % push-in over the last 6 s | label: `Node 1 → US Lab → Node 2 → Columbus` · caption (from 0:20): `Hand over hand along the handholds, contacts on` · caption (last 6 s): `Final reach to the service panel` | live `Handholds  N / 36` (N = captures so far from the render sidecar; 36 = `main_sv5_summary.total_steps`) · last 6 s: `Panel reach  2.5 cm` (`main_sv5_summary.panel_reach.min_distance_m`) | hard cut | 8× |
| 4 | 0:58–1:13 | `jem_sv5_run.json`, last 120 s of the route (ends at completion) | chase camera | label: `Node 2 → JEM` · caption: `Same controller, second route` | live `Handholds  N / 18` (sidecar; `jem_sv5_summary.total_steps`) | hard cut | 8× |
| 5 | 1:13–1:28 | `node3_sv5_run.json`, last 120 s of the route | chase camera | label: `Node 1 → Node 3` · caption: `Three routes, one code version, one run each` | live `Handholds  N / 16` (sidecar; `node3_sv5_summary.total_steps`) | 12-frame dissolve out (act boundary) | 8× |
| 6 | 1:28–1:40 | `handoff_run.json`, sim 12–24 s | existing oblique demo camera (behind-left, slow follow) | label: `Tool hand-off` · caption: `Astrobee releases a tool bar. G1 catches it.` | after the catch frame: `Release to catch  6.1 s` (`handoff_summary.catch.t_s − release.t_s`) | hard cut | 1× (no badge) |
| 7 | 1:40–1:56 | `pushoff_run.json`, sim 0–32 s | existing oblique demo camera | label: `Push-off` · caption 1: `G1 pushes off and lets go` · caption 2 (after the release frame): `Momentum holds until the first contact` | `CoM drift  0.088 m/s` (`momentum_conservation.com_speed_m_s`) · `Δp  ≤ 0.00018 kg·m/s over 4.25 s` (`max_dP_kg_m_s`, `window_s`) | hard cut | 2× |
| 8 | 1:56–2:11 | `render_cupola_earth.py` re-render, day pass into sunset only (orbit fraction chosen from a 6-frame preview; the Earth/Sun model is unchanged) | static oblique view from the Cupola position, as before | label: `Earth from the Cupola position` | `Orbit time  M min` live (phase × `ORBIT_S`, written per frame to a sidecar by the renderer) | hard cut | badge `≈ N×` = orbit seconds covered ÷ 15 s, computed |
| 9 | 2:11–2:27 | left: `fin_main_0g_v2_run.json`; right: `fin_main_1g_run.json`; both sim 0–16 s, rendered at 1920×1080 and centre-cropped to 960×1080 (no letterbox) | chase camera, same path definition for both | labels: left `0 g`, right `9.81 m/s²` · caption: `Same route and controller. Only gravity changes.` | left live `Handholds  N / 36` · right live `Handholds  0 / 36` (sidecars; `completed_steps`) · right, after the miss: `Miss at handhold 1  17.5 cm` (`fin_main_1g_summary.capture_error_m[-1]`) | 12-frame dissolve out (act boundary) | 1× (no badge) |
| 10 | 2:27–2:45 | lunar POV, new render from `scene_walk_rec.json` (P3 policy, seed 1000; existing rollout, no new simulation). Window: the last 4 touchdowns before the course's stop phase, found from `record.contact` | camera rigid on `head_link`, orientation low-passed (1.5 Hz cut-off, zero-phase), FOV 70°, pitch ≈ 15° down; on the last step it eases to 35° down over 3 s, holds while the print forms and dust falls, the foot lifts, then holds 2 s on the print | label: `Moon · learned walking policy` · caption: `Velocity tracking gate not met: 0.24 m/s RMS, limit 0.15` | readout: `g  1.62 m/s²` (`p3_moon_scratch_at_moon_eval.summary.gravity_m_s2`); caption numbers from `tracking_rms_m_s` and `gates.tracking_rms_max` | hard cut in | 0.5× |
| 11 | 2:45–2:46 | black | none | none | none | 8-frame fade to black | n/a |
| 12 | 2:46–2:52 | black card, centred | static | title (64 px): `1.62 m/s². No atmosphere.` / body (34 px): `Unitree G1 in NVIDIA Isaac Sim` | 1.62 from the same eval field as shot 10 | hard cut | n/a |
| 13 | 2:52–3:06 | credits on black, 22 px, left-aligned in two columns | static | credit lines below | 8206c91 has no run-file field; it is read from the receipt heading in `docs/ISS_TO_LUNAR_SHOWCASE_ARCHITECTURE.md` ("commit 8206c91") by regex, flagged as a doc source; no other numbers | fade out | n/a |

Total ≈ 3:06.

### Credits text (exact; each line ≤ 60 characters)

```
ISS routes: commit 8206c91, one run each, contacts on.
Repeatability across runs was not measured.
Grasps are scripted constraints (fixed joints).
Some handholds and the panel anchor are placeholders.
Hand-off, push-off and the 9.81 m/s² comparison start from
an earlier controller run (fin_main_0g_v2 state).
Astrobee propulsion is a force-limited stand-in.
Earth view is rendered; the Cupola mesh is hidden.
Footprints and dust launch are rendered effects.
Dust follows ballistic paths at 1.62 m/s².
Moon: Isaac Lab G1 (37 joints), not the ISS G1 (29 DoF).
ISS interior: NASA 3D Resources via nasa/astrobee_media.
Earth: NASA Blue Marble Next Generation.
Simulation: NVIDIA Isaac Sim 6.0.1, Isaac Lab, cuRobo.
```

### Lunar look fixes (render only; walk, gravity and the dust model unchanged)

- **Robot exposure.** Lower the Sun intensity and add camera exposure compensation, so the white shell stops clipping. In the POV the robot shows only its knees and feet.
- **Ground.** Cut the large-scale height noise that read as water ripples, and add fine crater/pebble detail (cell 0.015 m, as now). A new random seed for this noise is a look change, not a data change.
- **Bootprints.**
  - Add per-print jitter: ±4° yaw, ±15 % depth, tread phase.
  - Soften the rim with a 2-cell blur, so the prints stop looking stamped.
  - Footprint size stays as now.
- **Sky and lighting.**
  - Earth at about 4° elevation, ahead and slightly right, so it sits near the top of the frame at the 15° pitch.
  - Sun at 8° elevation from the left, for long side shadows.
- **Wording.** No Apollo line on screen; no "visual effect" text over the shot. The disclosure is in the credits.

## Cut from the current version, and why

- **Korean title, "to the Moon" and summary cards.** Replaced by the cold open, the English title and the credits. The cards carried meta language and mixed languages.
- **JEM | Node3 split screen.** Replaced by sequential 15 s shots. The split left half the frame black, and the per-panel text was unreadable.
- **Isaac Sim GUI recording and Korean explanation panel (gravity comparison).** Replaced by a full-height side-by-side shot. The GUI was cluttered, and "capture radius 4 cm" was a typed number with no field in a run file.
- **Night half of the Cupola orbit.** More than half of the old 30 s was black.
- **Third-person lunar walk (27.6 s).** Read as light and careless; replaced by the POV finale.
- **Burned-in "contact events this step" counts.** An unexplained internal number that changed every step.

## Number source table

The source field is read at compose time.

| On-screen | Value now | File | Field |
|---|---|---|---|
| `N / 36` (main) | 36 at end | `outputs/iss_activity/main_sv5_summary.json` + render sidecar | `total_steps`, captures per frame |
| `Panel reach 2.5 cm` | 0.0249 m | `main_sv5_summary.json` | `panel_reach.min_distance_m` |
| `N / 18` | 18 | `jem_sv5_summary.json` + sidecar | `total_steps` |
| `N / 16` | 16 | `node3_sv5_summary.json` + sidecar | `total_steps` |
| `Release to catch 6.1 s` | 21.004 − 14.904 | `outputs/iss_demos/handoff_summary.json` | `catch.t_s`, `release.t_s` |
| `CoM drift 0.088 m/s` | 0.0883 | `pushoff_summary.json` | `momentum_conservation.com_speed_m_s` |
| `Δp ≤ 0.00018 kg·m/s over 4.25 s` | 0.00018; 5.454 − 1.204 | `pushoff_summary.json` | `momentum_conservation.max_dP_kg_m_s`, `window_s` |
| `Orbit time M min`, badge `≈ N×` | per frame | Cupola render sidecar (new) | phase × `ORBIT_S` (script constant 5550 s, stated in the render's docstring) |
| `0 / 36`, `Miss at handhold 1 17.5 cm` | 0, 0.1753 m | `fin_main_1g_summary.json` | `completed_steps`, `capture_error_m[-1]` |
| `N / 36` (0 g side) | live | `fin_main_0g_v2_summary.json` + sidecar | `total_steps` |
| `g 1.62 m/s²`, final card | 1.62 | `outputs/lunar_v0/p3_moon_scratch_at_moon_eval.json` | `summary.gravity_m_s2` |
| `0.24 m/s RMS, limit 0.15` | 0.244, 0.15 | same file | `summary.tracking_rms_m_s`, `summary.gates.tracking_rms_max` |
| speed badges | 8×, 2×, 0.5× | compose script | derived from the render's `--every`/`--speed` and the shot's sim window |

## Render budget (estimate, not measured)

About 4,600 path-traced frames at 1080p:

| Shot | Frames |
|---|---|
| 3 | 1,310 |
| 4 | 450 |
| 5 | 450 |
| 6 | 360 |
| 7 | 480 |
| 8 | 450 |
| 9 | 2 × 480 |
| 10 | ≈ 600 |

Earlier 720p renders took roughly 1 s per frame. 1080p has 2.25× the pixels, so expect about 2–3 hours of GPU time, run detached and waiting for a free GPU.

## Open risks

- **Windowed renders.** `--start-s` runs of `render_iss_activity.py` set the chase camera from the whole record, so a window should frame the same as the full render. Inferred, not yet checked.
- **Cupola day pass.** Where the day pass sits in orbit phase is not known yet; a 6-frame preview decides it.
- **POV camera.** It may show the robot's own head or shoulders at the 35° pitch; the near clip plane will be set past the head mesh.

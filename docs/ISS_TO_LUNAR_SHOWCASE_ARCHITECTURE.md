# ISS → Lunar Showcase Architecture

Baseline reviewed: 2026-10-04, Windows canonical repository at commit `8d3a511`.

## Concept

**Evidence-Driven Space Physical AI**: build one OpenUSD/Isaac project that begins with ISS intravehicular microgravity and expands to lunar South Pole/base operations, while keeping **visual fidelity** and **physics validity** as separate tracks until a gate allows them to converge.

The project should look compelling because of physically meaningful geometry, lighting, motion, scale, and telemetry—not because of decorative sci-fi UI.

Current truth:
- The reset-safe harness and known-answer calibration are working.
- Gate v3 (2026-10-04, PGS solver, thresholds unchanged; see `docs/ISS_V0_MOMENTUM_GATE.md`): **G1 passes** calibration, Test A and Test B. **R2 fails** Test B.
- G1 is the zero-g physics candidate: momentum conservation PASS under gate v3 only. Contacts, handrail grasping, controllers and hardware inertias are not validated.
- R2 remains a clearly labeled visual/kinematic actor (ISS historical context).
- Under the original v1 gate (TGS) neither passed. Those results are kept as history.

## Two-track development

### Visual track
Purpose: LinkedIn-quality scenes, camera language, spatial scale, materials, lighting, and understandable task storytelling.

Allowed before physics convergence:
- NASA/Astra-inspired ISS visual geometry.
- R2 or G1 kinematic motion with an explicit `VISUAL-ONLY` or `KINEMATIC` label.
- Lunar South Pole terrain and lighting references.
- Mission phases and telemetry layout prototypes.

### Physics track
Purpose: minimum scenes, clean collision proxies, deterministic tests, controller/policy evaluation, and evidence receipts.

No visual mesh is required to pass a physics gate.
## OpenUSD / Isaac scene contract

Suggested scene organization:

```text
/World
  /Environment
    /ISS
      /Visual
      /Collision
      /Affordances
      /Lighting
    /Lunar
      /Visual
      /Collision
      /Terrain
      /Affordances
      /Lighting
  /Actors
    /R2_Visual
    /G1_Candidate
  /Sensors
  /Task
    /Targets
    /Waypoints
    /Contacts
  /Telemetry
  /Debug
```

Run evidence should remain a sidecar JSON/artifact rather than being embedded as scene truth. Each accepted asset records source URL, license/provenance, version or commit, file hash, scale, frames, collision strategy, inertial source, and uncertainty.

### Layer rules
- **Visual**: render mesh, materials, decals, cables, labels, Earth/Moon view.
- **Collision**: simplified engineering proxies only; never assume visual mesh is collision truth.
- **Affordances**: handrail/grasp surface, hatch, tool, panel, foothold, cargo, maintenance target.
- **Lighting**: physically motivated Sun/environment setup; lunar date/site metadata when South Pole lighting is used.
- **Sensors**: camera/depth/IMU/LiDAR only when required by a task.
- **Telemetry**: values generated from the actual run, never decorative fake numbers.

## Analysis boundary

### Isaac Sim / Isaac Lab
Use for rigid-body/articulation dynamics, zero/low gravity, contact, grasping, collision, sensors, controller tests, RL/IL, and reproducible task metrics.

### External / co-simulation
Keep high-fidelity orbital propagation, structural FEM, thermal/vacuum, radiation, and validated regolith constitutive models outside Isaac unless a specific verified coupling is added.

### Visualization only until validated
ISS decorative interior detail, full-station geometry, lunar habitat architecture, EVA exterior scenes, dust effects, cinematic particles, and conceptual base infrastructure.
## Scenario ladder

| Stage | Scene | Visible action | Engineering question | Evidence | Hero visual | Advance gate |
| --- | --- | --- | --- | --- | --- | --- |
| S0 — Harness | Minimal zero-g | box / 2-link conservation | Is the measurement trustworthy? | calibration PASS + reset isolation | clean vectors + compact plot | completed |
| S1 — ISS Micro-Lab | one short interior bay + 2 handrails | R2 visual actor floats between rails; physics actor remains explicitly separate | Can the scene contract separate visual and physics truth? | scale/collision/provenance receipt | wide ISS corridor, Earth light, handrail foreground | scene audit PASS |
| S2 — Zero-g Actor Gate | proxy environment only | prescribed internal arm motion | Does candidate conserve momentum within fixed thresholds? | P/L ratios + initial-state hash | split view: robot + P/L trace | candidate Test A/B PASS |
| S3 — Handrail Transfer | 2–4 rails | grasp → push/pull → flight → regrasp → park | Can contact locomotion be repeatable without excessive reaction? | success, impulse, peak force, slip, collision, duration | side view showing body arc between rails | deterministic controller baseline PASS |
| S4 — ISS Task | one panel/tool zone | park → manipulate tool/panel | Can stable positioning support manipulation? | pose error, force, collisions, task time | close technical shot with hand/tool | task metrics PASS |
| S5 — Lunar Rigid Patch | flat rigid ground, 1.62 m/s² | candidate walks waypoint course | What changes under lunar gravity? | fall/slip/tracking/torque/energy | low-angle Sun, long robot shadow | lunar rigid-ground gate PASS |
| S6 — South Pole Terrain | OmniLRS/LRO-derived tile | traverse slope/rocks/shadow | Does terrain + lighting alter mobility/sensing? | site/DEM/date provenance + mobility metrics | ridge line with deep shadow transition | terrain scenario PASS |
| S7 — Base Operations | small habitat/cargo/maintenance cluster | inspect → carry → service | Can one robot execute useful base operations? | task success, energy, collisions, mission time | habitat in background, robot foreground | single-robot ops PASS |
| S8 — Multi-Robot | only after S7 | logistics/construction coordination | Does coordination add measurable value? | throughput / energy / task latency | wide base operations shot | later research |

Do not build the full ISS or full Moon Base before S3/S6 proves the relevant physics.
## LinkedIn visual strategy

### 30–45 second first video
1. **0–4 s — Hero:** ISS micro-lab wide shot. Floating R2 silhouette, Earth light through a window/hatch. Small label: `VISUAL / KINEMATIC`.
2. **4–9 s — Truth layers:** same camera, briefly reveal visual mesh → collision proxy → grasp affordances.
3. **9–16 s — Validation:** zero-g internal actuation clip with only two overlays: `P` and `L`; show the fixed threshold and actual verdict.
4. **16–22 s — Engineering honesty:** compact calibration PASS / G1 PASS (gate v3) / R2 FAIL card. No dramatic red warning screen.
5. **22–29 s — Task future:** handrail transfer path as a thin trajectory arc; label `NEXT GATE`.
6. **29–37 s — Moon transition:** cut to lunar rigid patch, then South Pole terrain reference with low Sun and long shadows.
7. **37–45 s — End frame:** split-screen ISS / Lunar with text: `same evidence contract, different gravity regime`.

### Overlay rules
- Maximum 2–3 simultaneous data elements.
- Use real run values only.
- Keep camera motion slow and legible; no aggressive shake, bloom, lens flare, or fake HUD clutter.
- Use physically meaningful Sun/Earth lighting as the primary visual style.
- Every unvalidated robot shot carries a small persistent state tag: `VISUAL-ONLY`, `KINEMATIC`, or `CANDIDATE`.

### Hero frame
First release: a clean ISS interior bay, handrail in foreground, R2 floating mid-frame, Earth/blue light as depth cue, one small `ISS → LUNAR PHYSICAL AI` title. No dense dashboard.

The Moon should be a teaser in the first post, not half the first implementation.
## Asset sourcing and reuse

### ISS
- Use NVIDIA Astra/Omniverse Labs as an architectural and visual reference for OpenUSD ISS composition/telemetry.
- Prefer NASA 3D and Astrobee/ISS public assets as source/reference material after license/provenance review.
- Rebuild collision and grasp geometry as simple proxies.
- Do not import the full station for v0. One recognizable interior bay is enough.

### Robots
- **R2 now:** visual/kinematic storytelling, proportions, grasp pose studies, ISS historical context. Not current physics truth.
- **G1 now:** zero-g physics candidate. It passes gate v3 (momentum conservation, PGS) and is also the later lunar-walking candidate. Contact/handrail behavior is not validated.
- Before either becomes a validated ISS physics actor: pass reset-safe A/B under fixed thresholds with auditable inertial/collision provenance.

### Moon
- Begin with rigid lunar gravity patch.
- Move to OmniLRS/LRO-derived terrain after rigid-ground locomotion passes.
- Use NASA DUST / South Pole references for terrain and lighting provenance.
- Add a base only as a small operations cluster: habitat exterior, cargo zone, power/maintenance target, marked traversable corridor.
- Reuse robot/task/evidence schemas; rebuild environment physics where the regime differs.

## Minimum next implementation slice

**ISS Micro-Lab v0 Visual + Truth Layers**

Build only:
- one short ISS-inspired interior bay,
- two correctly scaled handrails,
- visual mesh layer,
- collision proxy layer,
- grasp-affordance metadata,
- one R2 visual/kinematic actor,
- camera/light rig,
- a small script that toggles Visual → Collision → Affordance views,
- a LinkedIn-ready 8–12 s hero clip.

Do **not** present the R2 motion as validated dynamics. In parallel, keep the zero-g actor gate as the physics track. Merge the tracks only after a candidate passes.
## S1 Implementation Receipt — ISS Micro-Lab v0 (2026-10-04)

> Superseded for the environment, R2 motion and video by **S1.1** below. This receipt is kept as the record of commit `20499db` (authored-primitive bay); its provenance statements apply to that commit only.

**Verdict: S1 scene-contract audit PASS (visual track only).** Nothing in this slice is a physics validation. R2 appears as `KINEMATIC / VISUAL`; the zero-g gate verdicts above are unchanged (calibration PASS, R2 reset-safe Test B FAIL, G1 reset-safe Test B FAIL).

| Item | Record |
| --- | --- |
| Runtime | Isaac Sim `6.0.1-rc.7+release.42383.32955d8d.gl` (`C:\isaacsim\VERSION`), Windows, RTX 4070 Ti 12 GB, driver 591.86 |
| Git before implementation | `bff0b09` (`main`) |
| Scene | `scenes/iss_microlab_v0.usda` — generated by `scripts/build_iss_microlab.py`; regenerate, do not hand-edit |
| Contract check | `scripts/check_iss_microlab.py` (pxr) and `--isaac` (opened in Kit): see results below |
| Renders | `scripts/render_iss_microlab.py` → `outputs/iss_microlab_v0/` (gitignored) |

### Scene hierarchy (as authored)

```text
/World                         VariantSet truthLayer = {visual, collision, affordance}
  /PhysicsScene                gravity magnitude 0 (IVA microgravity; no orbital mechanics)
  /Looks                       UsdPreviewSurface materials
  /Environment/ISS
    /Visual                    195 render gprims: 4 rack walls x 4 bays, corner standoffs, GLA luminaires,
                               cable runs, nadir window, aft bulkhead, forward hatchway + dim adjacent volume,
                               Handrail_A/B, MaintenancePanel
    /Collision                 29 primitive proxies (CollisionAPI): walls, 45-deg standoffs, bulkheads,
                               hatchway/vestibule, adjacent volume, Handrail_A/B (grip + standoffs), MaintenancePanel
    /Affordances               Handrail_A_Graspable, Handrail_B_Graspable, MaintenancePanel_Target
    /Lighting                  8 RectLight (GLA strips, 4800 K), NadirWindow_EarthLight (DiskLight), AdjacentModule, AmbientFill
  /Actors/R2_Visual            iss:classification = KINEMATIC_VISUAL_ONLY
    /R2                        reference -> outputs/r2_derived/isaac_import/r2c6_zero_g/r2c6_zero_g.usda
  /Task
    /Targets/MaintenancePanel  rel iss:affordance -> MaintenancePanel_Target
    /Waypoints/R2_Park         approach marker, rel iss:graspCandidates -> Handrail_B_Graspable
  /Cameras/Hero                19 mm / 36 mm aperture (~87 deg HFOV), no DOF; video adds a 0.12 m eased push-in
```

`/Sensors`, `/Telemetry`, `/Debug`, `/Task/Contacts` are intentionally not authored: no task in S1 needs them. `/Actors` and `/Task` carry no ISS-specific assumptions and can be reused for the lunar scenes.

### Layer representation
- One `truthLayer` VariantSet on `/World`; the same hero camera renders all three layers. Every layer-specific opinion (visibility, collision-proxy material) lives inside the variants, because a local opinion would always override a variant.
- **visual**: render geometry + R2. **collision**: proxies only (bounds neutral grey, task colliders amber); R2 hidden because it has no validated collision/physics model. **affordance**: proxies as dark context + graspable (teal), maintenance target (blue), approach waypoint (white).
- Affordance metadata is minimal (`iss:affordance` token + `iss:collisionProxy` / `iss:visualSource` relationships). No generic semantic framework was added.

### Provenance decisions
- **ISS environment: AUTHORED, ISS-INSPIRED.** All environment geometry is primitives written by the build script. No NASA, NVIDIA Astra or third-party ISS mesh was downloaded, referenced or copied. Generic ISS-interior conventions (four rack walls, corner standoffs with luminaires, nadir window, square hatchway) were used as visual reference only. This is not a digital twin; dimensions are approximate (2.2 m clear aisle, 1.05 m bays).
- **Astra / NASA ISS assets:** `REFERENCE_ONLY` (license/provenance for reuse not established in this slice).
- **Handrails:** authored; grip 0.45 m, section 33 x 24 mm, standoff 65 mm (assembly depth 89 mm). The checker only enforces a plausible range; an exact ISS IVA handrail specification source is UNKNOWN (not verified).
- **R2:** `outputs/r2_derived/r2c6_zero_g.urdf` SHA-256 `282152eee0c5fbf514f3cdf74b8ce842f659f32c634c7b9c147a7c0972a97d8f` (scripted derivative of `gkjohnson/nasa-urdf-robots@1f89ecb`, NASA-1.3 per `docs/R2_ASSET_AUDIT.md`); imported USD SHA-256 `361201984e1f33eba68303e1a9fd2602d95dd63283dae918a3699b45fa73345a`. In the scene layer only: articulation root removed (apiSchemas delete), `/R2/Physics` joints deactivated, all 130 rigid bodies `kinematicEnabled`. Pose = joint-orient overrides from a deterministic coordinate-descent IK within URDF limits (residuals: left palm 0.098 m short of the reach target by design, right palm 0.013 m, feet 0.018 / 0.021 m). The torso-shell texture carries agency/company insignia; the scene binds a plain shell material to those two meshes (asset files untouched). A small flag decal on the arm texture remains.
- No R2/G1 mass, inertia, armature, solver setting, joint topology, `configs/iss_v0_momentum.json` or gate threshold was modified.

### Render settings
RTX PathTracing, 1 spp per subframe, 64 subframes (stills) / 48 subframes (video), OptiX denoiser, 1920 x 1080, fixed exposure (auto-exposure off), no depth of field, motion blur or lens flare. Overlays are PIL typography only (title, `KINEMATIC / VISUAL` tag, layer indicator, one-line layer note); no telemetry numbers. Video: 300 rendered frames at 30 fps (10 s), slow eased 0.3 m dolly, layer cross-fades of 12 frames, H.264 CRF 17 yuv420p.

### Outputs (gitignored, `outputs/iss_microlab_v0/`)
`hero.png`, `visual_clean.png` (no overlay), `layer_visual.png`, `layer_collision.png`, `layer_affordance.png`, `iss_microlab_v0_linkedin_1080p30.mp4`, `render_receipt_*.json`, `scene_check*.json`.

### Verification commands
```powershell
C:\isaacsim\python.bat scripts/build_iss_microlab.py
C:\isaacsim\python.bat scripts/check_iss_microlab.py            # pxr contract checks
C:\isaacsim\python.bat scripts/check_iss_microlab.py --isaac    # same checks after opening in Isaac Sim (Kit)
C:\isaacsim\python.bat scripts/render_iss_microlab.py --mode stills --subframes 64
C:\isaacsim\python.bat scripts/render_iss_microlab.py --mode video --subframes 48   # needs ffmpeg on PATH
```
Prerequisite: the R2 derivative import under `outputs/r2_derived/` (see `docs/R2_DERIVATIVE.md` and the momentum-test importer).

### Verification results
- `check_iss_microlab.py` (pxr): **29/29 PASS** — required paths, metres/Z-up, zero-g physics scene, variant set + per-layer visibility, handrail dimensions, collision independence (29 proxies with CollisionAPI, 0 visual gprims with CollisionAPI, no references/instances), affordance and task relationships resolve, R2 classification / no articulation root / 0 active joints / 130 kinematic bodies, R2 conservative AABB clearance to aisle +0.038 m (head), R2 between bulkheads, no R2 overlap with handrail/panel proxies, camera and 11 lights resolve, `configs/` unchanged vs HEAD.
- `check_iss_microlab.py --isaac`: **29/29 PASS** — same checks after opening the scene in Isaac Sim 6.0.1 (Kit, headless).
- Video probe (ffprobe on the encoded file): h264, 1920 x 1080, yuv420p, 30 fps, 300 frames, 10.000 s.

### Visual QA (actual images inspected, including crops and frames decoded from the MP4: 0, 70, 100, 135, 150, 165, 195, 225, 255, 299)
Defects found and fixed: (1) variant material overrides ignored because base-layer bindings are stronger — moved all layer opinions into variants; (2) first pose read as an Earth-gravity squat/stride (hip abduction joint misread) — replaced with IK toward a bent-knee floating envelope; (3) left fingers penetrated the starboard wall / Handrail_B proxy — reach target moved inboard, no contact is authored; (4) head clipped the overhead corner — root lowered; (5) agency/company insignia on the torso texture — plain shell override; (6) Handrail_A outside the frame — moved into the foreground; (7) '→'/'·' rendered as missing glyphs (font coverage + an encoding round-trip) — fixed; (8) faceted window glazing — 64-segment mesh disc; (9) black hatch hole in the collision view — hatchway/adjacent-volume proxies added; (10) the first video's push-in cropped R2's head by the last frame — camera pulled back and push-in reduced to 0.12 m; re-rendered and re-inspected; (11) the Kit-side check found `NewtonArticulationRootAPI` still re-applying an articulation root on R2 (pxr alone does not register that schema) — the build now deletes every authored `*ArticulationRootAPI` token. This fix is non-visual: stills re-rendered from the final scene differ from the earlier ones only at path-tracing noise level (mean |diff| <= 0.7/255); the MP4 was rendered from the scene immediately before this schema-only fix.
Remaining accepted limitations: authored primitives read as clean CG rather than photographic ISS detail; the one-line layer note crosses the cable run at lower left (legible, not ideal); R2's helmet colour is the source asset's.

### Classification (unchanged by S1)
| Item | Visual | Kinematic | Physics validated |
| --- | --- | --- | --- |
| ISS Micro-Lab environment | yes (ISS-inspired, authored) | n/a | NO — collision proxies authored, never simulated in S1 |
| R2 (`/World/Actors/R2_Visual`) | yes | yes (authored pose) | NO — reset-safe Test B FAIL |
| G1 | not used in S1 | — | NO — reset-safe Test B FAIL (angular) |
| Calibration bodies | — | — | PASS (momentum harness only) |

### Known limitations / UNKNOWN
- Not a digital twin: bay dimensions, rack layout and handrail section are approximate; exact ISS IVA handrail specification UNKNOWN.
- The scene references the gitignored R2 import; a fresh clone must regenerate it before the scene resolves.
- R2 pose is authored by IK on visual links only; no self-collision check beyond visual inspection; hand-to-handrail contact is intentionally not authored.
- Collision proxies were not exercised in a physics run in S1; the affordance metadata is a placeholder for S3 (handrail transfer), not a validated grasp model.
- RTX path-traced appearance depends on driver/GPU; renders are not bit-reproducible across machines.

## S1.1 Receipt — NASA US Lab interior + R2 kinematic clip + Isaac Sim GUI walkthrough (2026-10-04)

**Verdict: S1.1 scene-contract audit PASS (visual track only).** S1.1 replaces the authored-primitive bay of S1 (commit `20499db`, receipt above kept as history) with the NASA US Lab interior mesh, keeps the truth-layer contract, and adds an authored R2 keyframe clip plus a scripted Isaac Sim GUI walkthrough. Still nothing here is a physics validation; the zero-g gate verdicts are unchanged.

### What changed vs S1
| Item | S1 (`20499db`) | S1.1 |
| --- | --- | --- |
| Visual environment | 195 authored primitives, ISS-inspired | `us_lab.usd` (NASA US Lab interior) + 2 × `handrail_30.usd`, referenced, not copied into git |
| Aisle used for collision/clearance | authored 2.2 m box | measured from the US Lab mesh: half-width 1.00 m, half-height 0.94 m, corner limit \|y\|+\|z\| ≤ 1.55 m, rack zone x ±3.30 m, hatch 1.27 m |
| Collision proxies | 29 | 33 primitive proxies (walls, 45° corner standoffs, bulkheads with hatch, hatch tunnels, handrails, panel) |
| R2 | one static IK pose | 8 s keyframe clip (5 keys, IK per key, smoothstep, sampled every frame 0–240 @ 30 fps): drifts aft along the aisle and reaches for Handrail_B |
| Video | 10 s, static pose, dolly | 12 s: visual 0–8 s (clip plays), collision, affordance, visual |
| Isaac Sim UI | none shown | `scripts/gui_demo_iss_microlab.py`: real Isaac Sim GUI screen recording |

### Provenance (S1.1)
- **ISS interior + handrails: NASA media; two license statements, governing terms UNKNOWN.** Source repo `https://github.com/nasa/astrobee_media` at `1cb0620121099cc848f5a3db480464188dc91ea3` (2023-03-20). `astrobee_iss/package.xml` and every `astrobee_handrail_*/package.xml` declare Apache-2.0, but the repository `readme.md` says all multimedia in it is covered by NASA's Media Usage Guidelines (https://www.nasa.gov/multimedia/guidelines/index.html). The interior meshes are redistributed from NASA 3D Resources "ISS Internal" (IGOAL, https://nasa3d.arc.nasa.gov/detail/iss-internal); the handrails from `nasa-jsc-robotics/r2_gazebo`. Both statements are recorded. Until clarified, treat the meshes as NASA media under the Media Usage Guidelines (no NASA endorsement or insignia implied) and do not redistribute them in Git. (Earlier revisions of this document said "Apache-2.0" only.)
  - `astrobee_iss/meshes/us_lab.dae` SHA-256 `1b54da04c84139eeb05944c92ef7bbf3cdaf6b7b92d0a3ac178ea911f0cc71aa`
  - `astrobee_handrail_30/meshes/handrail_30.dae` SHA-256 `0dc019783d46a353e4daea0cf0b664a8048c2d99138bef378f8588866bb28c5a`
- Converted by `scripts/import_iss_media.py` (`omni.kit.asset_converter`, metres) to `assets/cache/iss_media/` with `import_report.json`. `assets/raw/` and `assets/cache/` are gitignored; a fresh clone must sparse-clone `nasa/astrobee_media` into `assets/raw/astrobee_media` and run the importer.
- US Lab placement = the Astrobee simulator's own transform (rotateX 90°, scale 1.0858), recentred at x = 4.2 m. The appearance is a game-grade texture-baked mesh: this is **not** a high-fidelity or certified ISS digital twin.
- The aisle numbers above were measured from mesh vertices; only one corner was cleanly measurable and its value was applied to all four (approximation, recorded as such).
- R2 provenance unchanged from S1 (URDF / USD SHA-256 above). Same scene-layer-only physics removal and insignia override.

### R2 motion = authored animation
- `iss:classification = KINEMATIC_VISUAL_ONLY`, `iss:clip` describes the clip. Poses come from coordinate-descent IK on visual links within URDF limits; there are no forces, contacts or dynamics. Clip IK residuals: ≤ 0.025 m for all effectors except the left hand at 6.0 s / 8.0 s (0.067 / 0.058 m short of the grip; the hand is near the rail, not on it). No grasp or contact is claimed.

### Isaac Sim GUI walkthrough (scripted)
`C:\isaacsim\isaac-sim.bat --exec "scripts/gui_demo_iss_microlab.py"` opens the scene in the Isaac Sim GUI, pins its window on top and records only that window's client area with ffmpeg `gdigrab` (cursor visible; no recording if Isaac Sim is not in the foreground) to `outputs/iss_microlab_v0/gui_walkthrough_raw.mp4`. The viewport, Stage/Property/Console panels, timeline playback and variant switching are the real Isaac Sim UI; **the cursor path and the selection/commands are scripted**, and the Console lines are written by the script (`[iss-s1] …`). It is a reproducible demo, not a recording of manual operation or of a physics run.

### Verification results (S1.1)
- `check_iss_microlab.py` (pxr): **30/30 PASS**. Includes: visual layer = referenced NASA meshes (`us_lab.usd` + 2 × `handrail_30.usd`), 5 visual gprims without CollisionAPI, 33 independent collision proxies with CollisionAPI; handrail grip 0.70 m, section 34 × 25 mm, assembly 0.784 m (NASA mesh dimensions; checker range 0.30–0.90 m); clip authored (241 root samples, end time code 240, 30 fps); conservative AABB clearance to the measured aisle > 0 at frames 0–240 step 10 + default (min 0.014 m, `r2cl_ext_bone2` @ 240); R2 x within rack zone [0.025, 2.989] m; no R2 overlap with handrail/panel proxies; camera inside the aisle; 9 lights; per-layer visibility; `configs/` unchanged vs HEAD.
- `check_iss_microlab.py --isaac`: **30/30 PASS** (same checks after opening the scene in Isaac Sim 6.0.1 Kit, headless).
- Renders: `--mode stills --subframes 64` and `--mode video --subframes 48` exit 0. ffprobe on `iss_microlab_v0_linkedin_1080p30.mp4`: h264, 1920 × 1080, yuv420p, 30 fps, 360 frames, 12.000 s.
- GUI walkthrough: `gui_walkthrough_raw.mp4` 2560 × 1494 (Kit client area only), 33.6 s; `gui_walkthrough_1080p30.mp4` is the same scaled/padded to 1920 × 1080; `iss_microlab_v0_s11_render_plus_gui_1080p30.mp4` (45.6 s) = rendered clip followed by the GUI walkthrough.
- All renders, recordings and both checks above were produced from the same build output, `scenes/iss_microlab_v0.usda` as written by the build script (LF line endings) SHA-256 `417d013f79b8af345d61bf955e53d92ae1348e7bf74a4138c58f1661c3268d74` (build is deterministic: two rebuilds gave the same hash).
- Visual QA (images and frames decoded from the MP4s actually inspected): hero/layer stills; video frames every 30 frames; collision↔visual cross-fade frame 240 shows the handrail proxies on the NASA rails and the hatch proxy in the hatch; GUI recording sheet (every 2.7 s) shows R2 selected in Stage with Property values, Console trace, clip playing, collision and affordance layers, and the session-camera close-up.
- Defects found and fixed in S1.1: (1) R2 head 0.10 m into the overhead and legs past the rack zone → keys re-posed, start at x = 2.0 m; (2) the first GUI recording captured the full desktop instead of only Isaac Sim → deleted; the script now pins the Kit window on top, refuses to record unless it is in the foreground, and captures only its client area; (3) window-title capture returned a frozen Vulkan viewport → capture the composed screen region of the client area instead; (4) variant switches during the walkthrough dirtied the scene file in Kit → moved to the session layer (scene file unchanged, mtime verified).

### Known limitations / UNKNOWN (S1.1)
- Visual and collision are independent: proxies were placed from the measured aisle and inspected visually against the mesh; exact proxy-to-mesh agreement is approximate (corner value applied to all four corners).
- Handrail mesh orientation (standoff along local +X) was taken from the mesh bbox and visual inspection; the ISS IVA handrail specification itself remains UNKNOWN.
- R2 self-collision and hand-rail contact are not checked by physics; clearance is a conservative AABB check against the measured aisle every 10 frames.
- The rendered clip and GUI recording depend on the gitignored NASA media and R2 import being regenerated locally.

## S1.2 Receipt: physics in the ISS scene, three-panel clip (2026-10-04)

> Superseded by **S1.3** below. The S1.2 scripts were renamed and rewritten as `scripts/iss_physics_*.py`; S1.2 is kept as the record of commit `4319667`. S1.3 QA found that RTX kept instanced G1 hand meshes visible when an actor root was hidden per panel. The S1.2 clip may therefore show stray G1 hands in its cargo panel. Not re-checked.

**Verdict: PhysX actually runs in the ISS scene for G1 and the cargo. R2 stays kinematic. In-scene G1 momentum check: PASS (angular 0.00787, linear 0.0000039, gate v3 limits 0.01 / 0.001).**

| Panel | Actor | What drives it | Label |
| --- | --- | --- | --- |
| 1 | R2 | S1.1 keyframe clip (USD time samples), holds the last pose after 8 s | `KINEMATIC / VISUAL` |
| 2 | G1 (Unitree `g1_29dof.urdf`, SHA-256 `e1dc8936…`) | PhysX PGS, gate v3 physics settings, free-floating; arm PD sine exactly as gate Test B (shoulder ×3 + elbow per arm, 100/10, 0.5 rad, 0.5 Hz); all other joints zero gain | `PHYSICS · PhysX PGS` |
| 3 | 3 generic CTB-sized boxes (0.50×0.43×0.25 m, 10 kg; not a NASA cargo model) | PhysX rigid bodies, authored initial velocities, restitution 0.2 / friction 0.5, colliding with the S1.1 collision proxies (shown in the collision-layer view) | `PHYSICS · RIGID BODIES` |

- **Pipeline:** `scripts/s12_record_physics.py` opens `scenes/iss_microlab_v0.usda`, deactivates R2, references a fresh G1 URDF import, adds the cargo and collision groups, runs 2,400 PhysX steps (dt 1/240, 10 s), and records link/box poses at 30 fps plus G1 momentum every step to `outputs/s12/physics_record.json`. `scripts/s12_render_panels.py` replays those poses on a physics-disabled copy and path-traces three 640×1080 cameras per frame (32 subframes) into `outputs/s12/s12_three_panel_1080p30.mp4` (1920×1080, 30 fps, 301 frames, 10.03 s) and `s12_three_panel_still.png`. Shared layout: `scripts/s12_common.py`. Nothing is re-simulated during rendering.
- **G1 contact condition:** a collision group filters G1 against the environment proxies and the cargo, so G1 runs under the same no-contact condition as the gate. The minimum G1 link-origin clearance to the measured aisle over the run was 0.247 m. Link meshes were inspected visually only.
- **G1 telemetry in panel 2** is the running maximum of |ΔL|/Lref from this run (final 0.00787). It is not a constant copied from the gate (the gate's own G1 value is 0.00703). The difference comes from placement/orientation in the scene and is not investigated.
- **Observed G1 motion:** with total angular momentum ≈ 0, the pelvis reorients by up to ~87° and moves ~0.14 m over 10 s while the arms oscillate and the zero-gain legs/waist swing freely (internal reconfiguration; the COM does not drift by more than the linear tolerance). This is the simulated behavior of a floppy-jointed model, not a controller or an astronaut-like posture.
- **Not validated:** cargo contact physics (restitution/friction values are authored, not measured), any G1 contact, G1 hardware inertias, and the handrail task. Visual QA: one frame per second of the MP4 decoded and inspected; all three panels move and the labels stay legible.

## S1.3 Receipt: G1 posture hold + Handrail_B grasp, R2 as kinematic context (2026-10-04)

> Superseded by **S1.4** below. The S1.3 hold panel replayed the gate Test B arm sine. That is a test excitation, not an IVA task, and it read as dancing in review. S1.3 is kept as the record of commit `c760420`.

**Verdict:**
- G1 posture hold: attitude is stabilised (max pelvis turn 13.8° vs ~87° with zero-gain legs in S1.2), but the momentum ratio is **0.0184 > 0.01 → FAIL** for this condition.
- G1 handrail reach: the scripted grasp **ACHIEVED** at t = 1.77 s, held with a post-grasp hand drift ≤ 0.03 mm.
- R2 is shown only as the kinematic clip, because its physics model fails the momentum gate (Test B), not because a USD is missing.

| Panel | Actor | What runs | Label |
| --- | --- | --- | --- |
| 1 | R2 | S1.1 keyframe clip, holds the last pose after 8 s. Caption: R2 physics model fails the momentum gate, so physics is shown with G1 | `KINEMATIC / VISUAL` |
| 2 | G1_Hold | PhysX, gate v3 physics. PD 100/10 on **all 29 DOFs** toward the initial pose; arms add the gate Test B sine | `PHYSICS · PhysX PGS` |
| 3 | G1_Grasp | PhysX, gate v3 physics, posture PD on all DOFs. The right arm interpolates to a reach pose (0.5–2.0 s) with a 0.02 m/s approach drift. When the hand is within 12 mm of its planned grip pose, a **fixed joint world↔right hand is created** (scripted grasp) | `PHYSICS + SCRIPTED GRASP` |

- **Reach pose:** solved before the recorded run by coordinate descent using PhysX itself as forward kinematics (teleport + one step). Right shoulder pitch/roll/yaw + elbow = [-0.953, -0.25, 0.008, 0.836] rad; hand-origin target in the pelvis frame [0.4, -0.16, 0.22] m, residual 4.6 mm. G1_Grasp is then placed so that, at that pose, the hand origin sits 0.06 m aisle-side of the Handrail_B grip axis (derived from the affordance prim). At the grasp the hand origin was 7.2 cm from the grip axis.
- **Contacts:** both G1s are collision-filtered against the environment and each other. There is no finger, palm or rail contact physics. The grasp is a constraint, and the hand mesh can overlap the rail. The hold-condition momentum ratio is reported as measured (running value in panel 2). It is not a gate verdict, because the gate drives only the arm DOFs. Stiff PD on all joints is a different condition and fails the tolerance here; tuning it to pass was not attempted.
- **Pipeline:** `scripts/iss_physics_record.py` (IK pass, then the recorded 10 s run, 2,400 PhysX steps, poses at 30 fps) writes `outputs/iss_physics/physics_record.json`. `scripts/iss_physics_render.py` replays it on a physics-disabled, de-instanced copy and path-traces three 640×1080 cameras (32 subframes) into `outputs/iss_physics/iss_physics_three_panel_1080p30.mp4` and `iss_physics_three_panel_still.png`. Shared layout: `scripts/iss_physics_common.py`. Outputs are gitignored.
- **QA findings fixed:** (1) the first capture threshold attached the hand 8.5 cm short of the rail; it was tightened to the planned pose ±12 mm, with a 6 cm palm standoff. (2) RTX rendered instanced hand meshes of a hidden G1 in another panel; the replica is now de-instanced. (3) The panel-3 camera was moved to a side view along the wall, so the hand–rail relation is readable.
- **Not validated:** grasp mechanics (scripted constraint), G1 hardware inertias, the posture controller (plain joint PD, no balance or attitude control), and anything involving contact.

## S1.4 Receipt: G1 IVA worksite task (anchor on handrail, reach the panel) (2026-10-04)

**Verdict:**
- **Right hand anchored:** the scripted grasp on Handrail_B is ACHIEVED at t = 1.96 s, 0.3 mm from the planned grip point; post-grasp hand drift ≤ 0.02 mm.
- **Left hand at the panel:** it reaches the maintenance panel point (min 0.9 cm, final 1.7 cm; no contact).
- The arm-sine excitation is removed from the video. R2 is still kinematic-only, captioned with the real reason (it fails the momentum gate).

| Time | G1 action (one PhysX G1, gate-v3 physics, PD 100/10 posture on all DOFs, contact-filtered) |
| --- | --- |
| 0–0.5 s | Floats toward the starboard rack (0.035 m/s initial drift) |
| 0.5–2.0 s | Right arm reaches Handrail_B. At the closest approach (within 4 cm of the plan) a fixed joint world↔right hand is created: **scripted grasp, not finger contact** |
| 3.0–5.0 s | Anchored by the right hand, the left arm reaches the maintenance panel (affordance `MaintenancePanel_Target`) |
| 5.0–10 s | Holds the worksite posture |

- **Reach poses:** IK (coordinate descent, PhysX as forward kinematics). Right residual 1.0 mm, left residual 14.2 mm. Targets are 6 cm aisle-side of the rail axis and the panel face.
- **Start pose:** extending the arm pushes the floating body back, so the start pose is shifted by the measured miss of short trial reaches. Closest approach per trial: [57.2, 0.0] mm. This is feed-forward placement, not a feedback controller.
- **Panels:** (1) R2 kinematic clip; (2) G1 worksite (visual layer); (3) the same replay in the affordance layer: teal graspable rails, blue panel target. Outputs: `outputs/iss_physics/iss_physics_three_panel_1080p30.mp4`, `iss_physics_three_panel_still.png` (gitignored).
- **S1.4 Isaac Sim GUI run (live PhysX, not a replay):** `scripts/gui_demo_iss_physics.py` (run with `isaac-sim.bat --exec`) opens the scene in the Isaac Sim GUI. All edits go to the session layer: R2 off, G1 at the recorded start pose, gate-v3 PhysX settings. It then presses Play and runs the same controller in a PhysX step callback, using IK targets from the record. Result: the scripted grasp fired at t = 1.96 s, 0.0 cm from plan, matching the headless record. The Console shows the scripted command trace (`[iss-s1.4] …`), and the cursor path is scripted. Only the Kit client area is captured (window pinned on top). In the GUI the 10 s of simulated time took roughly 25 s of wall time; the recording is real time, not sped up.
  - QA on the raw recording found two problems after the pause. (1) A desktop notification toast covered the bottom-right corner from 38.75 s. (2) Switching the truth-layer variant while paused made PhysX log `Removing a CollisionGroup while the simulation is running`.
  - The published cut `outputs/iss_physics/gui_physics_1080p30.mp4` is therefore trimmed to 0–38.5 s. The raw file with the toast was deleted, and the layer switch was removed from the script.
  - (3) On quit, Kit saved root-layer edits (timeline end time 1800, bound viewport camera) into `scenes/iss_microlab_v0.usda`. The file was restored from git and rebuilt (hash back to the build value). The script now stops the timeline and reloads the root layer before quitting. The published recording came from the pre-fix script; these fixes do not change the trimmed 0–38.5 s portion.
  - `iss_physics_panels_plus_gui_1080p30.mp4` (48.5 s) = three-panel render followed by the GUI run.
- **G1 asset setup (fresh clone, canonical since 2026-10-16):** sparse-clone `unitreerobotics/unitree_ros` @ `5994d4f` (`robots/g1_description`) into gitignored `assets/raw/unitree_ros_5994d4f/`, or set `ISS_G1_URDF`. The canonical file is `g1_29dof_rev_1_0.urdf`, SHA-256 `c0ae739c…`; the scripts record the actual hash. The S1.4 run in this receipt used the former `g1_29dof.urdf` (`ccfc6fd`, `e1dc8936…`, now marked Deprecated upstream). The rev_1_0 re-run is recorded in [ISS_V0_MOMENTUM_GATE.md](ISS_V0_MOMENTUM_GATE.md) ("Canonical G1 switch").
- **Render resume:** `iss_physics_render.py --resume` keeps existing frames and renders only the missing ones. It was added after a full render stopped at frame ~150 with exit code 0 and no traceback. The cause is UNKNOWN; another Kit process (`compose_idea_video.py`, not part of this repo) started at the same moment. The re-render completed (301 frames).
- **Not validated:** grasp mechanics and any hand/rail/panel contact; posture control (plain joint PD); G1 hardware inertias. The posture-PD momentum behaviour measured in S1.3 (ratio 0.0184 > 0.01) still applies: whole-body PD is outside the gate-validated condition.

## Full ISS interior v0 receipt (2026-10-15)

**Verdict (2026-10-16, with authored hatch passages):** `scripts/check_iss_full.py` gives **33/34** in both pxr and `--isaac` (Kit, headless).
- 28 PASS on source data.
- 5 **PASS_WITH_AUTHORED_PROXY**: hatch gaps bridged by authored passages and reported separately from source-mesh PASS.
- 1 FAIL: Node3–Cupola, UNKNOWN.
- Source-mesh hatch alignment alone still fails all six axial gaps. The predeclared tolerances (centre ≤ 0.05 m, |gap| ≤ 0.10 m) are unchanged.
- First build (2026-10-15, no passages): 27/33.
- Visual track only: no physics actor or physics run uses this scene yet.

**Scene and source**
- Built by `scripts/build_iss_full.py` (pxr only; regenerate, do not hand-edit) as `scenes/iss_full_v0.usda` (LF, SHA-256 `c7d70481be302f969aff1e89ae58fcb996d26006f2d947ecba74e0cc26577249`; 2026-10-17, with the authored passages and clearance-placed route anchors; the 15-step evidence run used `8db2d16c…`; earlier builds: passages only `7c1ac76e…`, first build `d40d7bd3…`). It reuses the S1 helpers from `build_iss_microlab.py`.
- Meshes: `scripts/import_iss_media.py` now converts all 7 interior modules plus handrails 8.5 / 21.5 / 30 / 41.5. Existing conversions are kept unless `--force`.
- Source: `nasa/astrobee_media@1cb0620`. The sparse checkout was extended to `astrobee_handrail_8_5` and `astrobee_handrail_41_5` at the same commit.

**Frames and poses**
- Every module is posed exactly by its `<visual>` origin in `astrobee_iss/urdf/model.urdf` (matrix error 0 in the checker). The URDF rpy uses `3.1415`, not π: about 0.4 mm at 4 m.
- `/World/Environment/ISS` rotates the Astrobee world frame (x forward, y starboard, z nadir) by 180° about x, so scene +Z is zenith. Renders confirm that the "DECK" labels and the nadir window point down.
- US Lab uses scale 1.0858 (from the URDF). Its URDF collision origin is 0.104 m forward of its visual origin. Both are recorded, the visual origin is used, and which one is correct is flagged UNKNOWN.

**Finding: the S1 micro-lab US Lab is posed 90° off from the Astrobee pose.**
- `omni.kit.asset_converter` writes `upAxis = Y` but keeps the DAE's Z-up coordinates: raw DAE vertices × 0.1 match the converted USD bbox within 1 mm.
- S1 therefore posed `us_lab.usd` with rotateX 90° and without the URDF's rotateX 180°. This differs from the Astrobee pose by 90° about the module axis.
- The full scene carries all S1 US Lab elements (rails, proxies, GLA lights, panel) through S1⁻¹ · URDF, so they stay on the same mesh surfaces. S1's wall names (port / starboard / overhead / deck) are therefore *not* station directions.
- `scenes/iss_microlab_v0.usda` was not changed.

**Collision (strategy recorded per module in `iss:collisionStrategy`)**
- US Lab: the S1 primitive proxies minus the two hatch caps, so both hatchways are open.
- Node 1 / Node 2 / Node 3 / Columbus / JEM PM / Cupola: static triangle mesh of the interior mesh (`approximation = none`), with no SDF, no convex decomposition and no rigid bodies.
- Collision fidelity of these meshes is UNVALIDATED (visual mesh used as a collider), and no physics run has used them.

**Affordances**
- The two S1 handrails and the maintenance target, US Lab only. No public ISS handrail layout exists, so no rails and no placeholders were added elsewhere.
- Six `hatch` affordances at the measured hatch centres.
- Known gaps, recorded on the ISS prim: PMM, Quest airlock, JEM ELM-PS, Russian segment, PMA-2/3 and visiting vehicles, and handrails outside the US Lab.

**Hatch alignment**
- Method: the enclosed line-of-sight opening through a 0.5 m slab at each module end, 1 cm raster.
- Tolerance, fixed before measuring: centre offset ≤ 0.05 m, |axial gap| ≤ 0.10 m.

| Hatch | Centre offset m | Axial gap m (+ = gap) | Openings m |
| --- | ---: | ---: | --- |
| Node1–USLab | 0.031 | 0.233 | 1.26×1.26 / 1.27×1.26 |
| USLab–Node2 | 0.013 | 0.135 | 1.27×1.27 / 1.27×1.26 |
| Node2–Columbus | 0.045 | 0.239 | 1.26×1.26 / 1.20×1.20 |
| Node2–JEM PM | 0.038 | 0.202 | 1.26×1.26 / 1.26×1.26 |
| Node1–Node3 | 0.010 | 0.411 | 1.26×1.26 / 1.21×1.20 |
| Node3–Cupola | **0.125** | 0.205 | 1.20×1.20 / **1.76×1.75** |

- All axial gaps fail. With the US Lab at its URDF collision origin, Lab–Node2 closes to 0.031 m but Node1–Lab opens to 0.337 m. The Node1–Node2 total stays about 0.37 m, so neither origin closes both.
- The likely reason is that the meshes omit the berthing vestibules (inference, UNKNOWN).
- The Cupola-side opening (1.76 m) probably measured a window aperture rather than its hatch, so that row is UNKNOWN.

**Authored hatch passages (2026-10-16, owner decision)**
- Module poses are unchanged (still exactly the Astrobee URDF).
- Each measured hatch gap is bridged by a square tube (`Passage_<A>_<B>`) with these elements:
  - four primitive box walls (0.03 m) in `Visual/Passages` (plain material `AuthoredProxyPassage`);
  - the same walls with `CollisionAPI` in `Collision/Passages`;
  - a `passage` affordance marker with its inner opening, length and status.
- Inner opening = the smaller measured opening of the two modules per lateral axis. Centre = midway between the two measured hatch centres. Length = the measured gap.
- Every passage prim carries `customData provenance = "AUTHORED_PROXY: CBM vestibule missing from source mesh; dimensions from measured hatch openings, not NASA drawings"`. The checker verifies this on all 66 prims.

| Passage | Inner opening m | Length m | Centre residual vs both hatches m | Checker |
| --- | --- | ---: | ---: | --- |
| Node1–USLab | 1.26×1.26 | 0.233 | 0.015 | PASS_WITH_AUTHORED_PROXY |
| USLab–Node2 | 1.27×1.26 | 0.135 | 0.007 | PASS_WITH_AUTHORED_PROXY |
| Node2–Columbus | 1.20×1.20 | 0.239 | 0.022 | PASS_WITH_AUTHORED_PROXY |
| Node2–JEM PM | 1.26×1.26 | 0.202 | 0.019 | PASS_WITH_AUTHORED_PROXY |
| Node1–Node3 | 1.21×1.20 | 0.411 | 0.005 | PASS_WITH_AUTHORED_PROXY |
| Node3–Cupola | 1.20×1.20 | 0.205 | **0.063** | **FAIL, status UNKNOWN** (hatch centres 0.125 m apart; the Cupola opening may be a window) |

- The passage-end residual is 0 by construction (the ends are the measured planes). Only the centre residual is an independent check.
- Re-rendered: `USLab_to_Node2_ThroughHatch.png` (visual) and `USLab_to_Node2_ThroughHatch_collision.png`, `Node1_Wide_collision.png` (collision).
- The square passage meets Node 2's rounded hatch frame with small visible corner gaps. This is visual only; the collision walls are continuous.
- No CBM drawings were used. Passage length and shape are placeholders until a NASA source is found.

**Runtime (Isaac Sim 6.0.1, RTX 4070 Ti 12 GB, driver 610.88, headless)**
- Scene opens in about 1 s.
- Whole-GPU `memory.used`: 2,227 MiB before the app, 4,058 MiB empty app, 4,524 MiB with the scene open, 5,428 MiB after rendering two cameras.
- `app.update()` rate with one 1920×1080 render product, renderer `RealTimePathTracing`:
  - Node1 view: 72.6 FPS stopped, 53.3 FPS with the timeline playing (static colliders only);
  - through-hatch view: 70.1 / 53.0 FPS.
- GUI viewport FPS was NOT measured.
- The renderer reported `RealTimePathTracing` although `PathTracing` was requested. Stills are 64 subframes in that mode.

**Renders (`outputs/iss_full_v0/`, gitignored)**
- `<Module>_Wide.png` for the 7 modules and `USLab_to_Node2_ThroughHatch.png`, plus `*_collision.png` and `contact_sheet_{visual,collision}.png`.
- Module lights are presentation sphere lights on each module's centre line, not an ISS lighting specification.
- QA defect found and fixed: rendering several cameras in one process by retargeting one render product overexposed every camera after the first. The first batch was kept in `qa_first_render/`. Each camera is now rendered in its own process (`--camera`). The perf FPS run still used one process with two cameras (frame rate only, no image output).

**Not done / UNKNOWN**
- No physics actor in this scene. Triangle-mesh contact behaviour is UNKNOWN.
- Hatch vestibules, the US Lab origin and the Cupola hatch measurement: see above.
- License: see the S1.1 provenance note (Apache-2.0 vs NASA Media Usage Guidelines, governing terms UNKNOWN).

## Full-ISS activity v0: G1 hand-over-hand traverse (2026-10-16)

**Verdict: PARTIAL.**
- PhysX-executed hand-over-hand traverse: **15 of 36 steps captured**. That is Node1 → hatch and authored passage → US Lab, past Handrail_B, to x ≈ 4.2 m (about 7.2 m of the 16 m route).
- Step 16 failed: the hand ended 0.121 m from its anchor after 4 corrective replans.
- Node2 and Columbus were not reached.
- The panel reach failed in execution: 0.21 m minimum hand distance, although planned to 8.7 mm.
- In 68 % of the recorded frames, cuRobo's sphere model of the executed G1 pose is within 1 cm of the ISS meshes or penetrating them. Contacts were filtered, so G1 brushes or clips module geometry. Penetration depth is UNKNOWN.

### Tool evaluation

**cuRobo — adopted as an external planner.**
- Source: `NVlabs/curobo` V2 @ `78fd485` (2026-09-10), Apache-2.0. Installed in a separate uv venv outside the repo and outside Isaac's python: torch 2.14+cu130, RTX 4070 Ti, driver 610.88.
- Windows is not in the official install guide (Ubuntu only). On this machine the V2 runtime-compiled kernels passed its IK self-test: single 0.000 mm, batch 93/100, collision-free 50/50.
- It ships `g1_29dof_rev_1_0`. Diffed against our canonical file, it adds `base_link` and differs in the `mid360` sensor frame; the 29 joints and all inertials are identical.
- Use here: floating-base, collision-aware IK against the exported ISS meshes.
- Workarounds needed, all recorded in `scripts/plan_iss_activity.py`:
  - Planning frame rotated −90° about y: the base chain X→Y→Z is singular at head-first poses.
  - CUDA graphs off: graph replay crashed with an illegal address after criteria and seed changes.
  - Self-collision off: cuRobo's G1 sphere model self-collides at q = 0; Isaac also runs without self-collision. Feasibility is computed from the joint-limit and world-collision constraints only.
  - Solver seeds reset per call, which makes plans deterministic (verified: identical repeat runs).

**SOMA (`NVIDIA/soma-retargeter`, Apache-2.0) — not adopted for ISS.**
- It is a kinematic retargeter of 1 g ground motion capture (BVH) to G1.
- Its bundled motion data use an NVIDIA evaluation license.
- No zero-g IVA motion data exist, so it does not fit this phase. It is a candidate for lunar walking (S5).

### What was tried first and failed: ballistic flights

- Method: push off an anchor, free drift, then a scripted capture at the next sparse anchor (hatch rims and the two S1 rails).
- Five variants all failed to capture: head-first pose, consistent launch plans, orientation hold at release, Jacobian-transpose terminal reach, and the rotated planning frame.
- Why: the free-floating G1 with PD-held legs leaves the anchor with 0.1–0.2 rad/s of residual rotation. Over 10–20 s flights the tucked or straight legs hit hatch frames, or the hand missed by 0.07–2.2 m.
- Last record: `outputs/fail_fix/drive_ballistic_last.log`.

### Method that partly works: closed-loop hand-over-hand

**Route anchors** — `build_iss_full.py`, affordance layer only, 37 anchors, maximum gap 0.97 m:
- 4 `hatch_rim` anchors on the top edge of the measured hatch openings.
- The S1 `Handrail_B`.
- 32 `graspable_placeholder` anchors, 0.15 m below the overhead surface found by ray-casting the source meshes along the route centre line. Each carries `customData provenance = "PLACEHOLDER: no public ISS handrail layout; …"`.
- `check_iss_full.py`: 34/35 (29 PASS, 5 PASS_WITH_AUTHORED_PROXY, 1 FAIL = Node3–Cupola, unchanged).

**Planner**
- `scripts/plan_iss_activity.py --serve`, a local cuRobo service.
- Inputs come from `scripts/export_iss_world.py`: 61 colliders, 141,056 triangles, plus the anchors.
- Legs are locked in a tuck (hip pitch −0.6, knee 1.2, ankle pitch −0.6 rad), because a 1.3 m G1 cannot pass 1.26 m hatches straight.
- Each step:
  1. Local IK from the current joints; otherwise global multi-seed candidates with an actuated-joint swing ≤ 0.8 rad, held hand ≤ 1 cm, free hand ≤ 4 cm, collision-free.
  2. Intermediate frames from IK seeded by joint interpolation, falling back to interpolation where that IK fails.
  3. The pelvis orientation goal moves only 30 % toward head-first per step.

**Executor**
- `scripts/iss_activity_run.py`. Physics is gate v3: PGS, dt 1/240, 16/4 iterations, zero gravity, damping and friction. Joint PD 300/30 on all DOFs (S1.4 used 100/10, which lagged too much).
- For every step it asks the planner to replan from the **measured** state, tracks the plan for 4 s, then holds 1 s.
- If the free hand is more than 4 cm from its anchor, up to 4 corrective replans follow.
- Capture is a scripted world fixed joint at the hand; the previous anchor joint is then disabled.
- `--filter-contacts` collision-filters G1 against the ISS colliders, as in S1.3/S1.4.
- End-to-end runner: `scripts/iss_activity_drive.py`. Replay renderer: `scripts/render_iss_activity.py`.

### Evidence run

`outputs/iss_activity/chain_online_run_r4.json` (gitignored); scene SHA-256 `8db2d16c…`, G1 `c0ae739c…`.

| Metric | Value |
| --- | --- |
| Steps captured / attempted / route | 15 / 16 / 36 |
| Simulated time | 173.5 s |
| Capture error, median / max | 16.7 / 39.8 mm (radius 40 mm) |
| Corrective replans | 24 in total (0–4 per step) |
| Pelvis error vs. plan after a move, median / max | 0.17 / 0.71 m |
| Steps whose final plan met all criteria | 7 of 16 (the rest used the smallest-swing fallback) |
| Highest link speed at a capture | 0.41 m/s |
| Panel reach from Handrail_B | planned 8.7 mm; executed minimum 0.21 m (FAIL) |
| Interference (cuRobo sphere model, legs at the tuck) | 3,537 / 5,205 frames within 1 cm or penetrating (68 %); mostly during moves |

A rerun with up to 8 corrective replans stopped silently after step 13: Kit exited without a traceback and without writing its result. The cause is UNKNOWN; it resembles the earlier unexplained S1.4 render stop.

### Not validated / UNKNOWN

- Grasps are scripted fixed joints; there is no finger, palm or handhold contact.
- Placeholders are virtual handholds, not ISS hardware.
- G1–ISS contacts were filtered, and the interference above shows that the executed motion is not collision-free. Penetration depth is UNKNOWN.
- Whole-body joint PD lies outside the momentum-gate condition (gate doc, 1b).
- Planner and executor disagree by a median of 0.17 m in pelvis position, from closed-chain PD tracking of kinematic plans. A torque-level or whole-body controller would be needed.
- Steps 16–36, the Node2/Columbus segment and the panel task were not completed.
- Rendered clip: `outputs/iss_activity/iss_activity_traverse.mp4` (1280×720, 10 fps, 1,735 frames, 173.5 s = simulated time). It is a replay of recorded link poses on a physics-disabled replica, labeled with these caveats.
  - QA: in the first render a free chase camera left the aisle, and 879 of 1,735 frames were black. The camera is now held on the module centre line, leaving 17 dark frames (1 %): the first frame and a short stretch at the Node1 hatchway.
  - The frames visibly show the G1 pressed close to the US Lab overhead racks, consistent with the interference figure. The tucked legs sometimes fill the foreground.

### Collision-aware planning, contacts ON (2026-10-17)

**Verdict: PARTIAL.**
- With G1–ISS contacts **on** (resolved by PhysX), the traverse is collision-free over the steps it completes, but it gets **only 4–5 of 36 steps**: Node1 through the hatch into the aft US Lab.
- It stalls when the trailing hand must pass the leading hand. No collision-free, small-swing closed-chain reach is found from those poses.
- The contacts-filtered run above still reaches further (15 steps) but clips the structure.

**What changed**
- **Measurement:** per-link penetration depth replaces the unresolved "UNKNOWN depth". It is the cuRobo G1 sphere-model radius minus the distance to the nearest ISS triangle (`plan_iss_activity.py` `penetration()`, mode `check`).
- **Planner** (`plan_iss_activity.py`):
  - 4 cm world-collision activation margin in the optimizer.
  - Feasibility judged by actual geometry: no sphere penetration, plus joint limits. The activation band had frozen the planner whenever the measured pose sat inside it.
  - Pelvis goal at the aisle centre (centre line + 0.15 m), not 0.55 m below the overhead anchors.
  - Every path frame is checked for penetration.
  - Largest joint change per step 1.5 rad, executed at 3 s per rad (4–8 s).
  - Held-hand acceptance 2 cm. The warm-started solver lands at 1.1–1.6 cm, and the fixed joint holds the hand in execution.
  - When no direct solution exists, a continuation march takes IK-consistent increments toward the anchor; a full reach with the pelvis held is tried first.
- **Executor** (`iss_activity_run.py`): contacts on by default (`--filter-contacts` restores the S1.3/S1.4 condition); slower moves per the planner's suggested duration; up to 10 corrective replans.
- **Anchors** (`build_iss_full.py`):
  - Placeholders are lowered toward the module axis until their 3-D clearance to every collider is at least 0.20 m. Vertical ray placement had left them 0.11–0.15 m from rack faces and the NASA mesh's own handrails, and the first clearance check missed the US Lab proxy boxes.
  - The Handrail_B hand target is 0.10 m on the aisle side of the grip axis.
  - The maximum anchor gap grew to 1.28 m, at the high Node2-centre and Columbus ceilings. `check_iss_full.py` is now **33/35**: the route-anchor check FAILs (gap > 1.0 m, limit unchanged), alongside the known Node3–Cupola FAIL.
  - Capture radius (4 cm) unchanged.

| Run | Contacts | Steps captured | Frames penetrating (any / > 1 cm / > 3 cm / > 5 cm) | Max penetration | PhysX contact events |
| --- | --- | ---: | --- | ---: | --- |
| `chain_online_run_r4` (above) | filtered | 15 | 68 % / 57 % / 27 % / 8 % | 0.105 m (torso) | n/a (filtered) |
| `cf1` | on | 6 | — | — | 0 in steps 1–6; 8,908 in failed step 7 (wrist, head) |
| `cf7` | on | 5 | — | — | 0 in all completed steps |
| `cf8` (current code) | on | 4 | 10 % / 7.5 % / 0 % / 0 % | 0.027 m | 5,300, all in failed step 5 (left ankle 2,291, head 1,645, right wrist 1,289) |

Penetration is sampled every 3rd recorded frame. cf8's numbers include its failed step.

**Not done / UNKNOWN**
- Steps beyond 5 with contacts on.
- A closed-chain trajectory optimiser (cuRobo trajopt with the held-hand pose as a constraint) or a whole-body controller, either of which would replace joint PD tracking of IK waypoints. Both are NOT_RUN.
- Hand–handhold contact remains a scripted fixed joint.
- The sphere model is cuRobo's approximation of G1's geometry, and its legs are fixed at the planner's tuck.

## Claim ladder

### Allowed now
- “Built a reset-safe zero-g momentum validation harness in Isaac Sim.”
- “Known-answer calibration bodies pass the fixed conservation thresholds.”
- “R2 still fails the internal-actuation test; I am keeping that failure visible rather than tuning the threshold.”
- “G1 conserves whole-body momentum within fixed thresholds in a free-spin and an internal-actuation test (Isaac Sim PhysX, PGS solver, gate v3).” This is momentum conservation only. Do not call it a zero-g-validated humanoid or a validated handrail controller.
- “Building an OpenUSD ISS-to-lunar research sandbox with separate visual and physics layers.”

### Allowed after a candidate passes the zero-g gate
- “Validated momentum conservation for this model under the defined internal-actuation test and fixed thresholds.”
- Still do not say “space-qualified,” “flight validated,” or “physically accurate ISS humanoid.”

### Allowed after handrail/contact task validation
- “Demonstrated repeatable simulated handrail transfer under defined contact/impulse/slip metrics.”

### Allowed after lunar locomotion validation
- “Evaluated lunar-gravity locomotion on the stated rigid/terrain model with recorded tracking, slip, torque and energy metrics.”

### Only after hardware / external validation
Use “sim-to-real validated,” “flight validated,” or equivalent only when supported by actual hardware or on-orbit evidence.

## Visual source references
- NVIDIA Astra / Omniverse Libraries ISS: https://nvidia-omniverse.github.io/omniverse-labs/projects/astra-omniverse-libraries/
- OmniLRS: https://github.com/OmniLRS/OmniLRS
- NASA Lunar South Pole environment: https://www.nasa.gov/reference/moonbase-environment/
- NASA DUST: https://software.nasa.gov/software/MSC-27522-1

This document is the single canonical visual/scenario architecture. Detailed experiment truth remains in the existing gate and diagnosis documents.

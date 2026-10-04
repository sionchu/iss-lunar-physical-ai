# ISS → Lunar Showcase Architecture

Baseline reviewed: 2026-10-04, Windows canonical repository at commit `8d3a511`.

## Concept

**Evidence-Driven Space Physical AI**: build one OpenUSD/Isaac project that begins with ISS intravehicular microgravity and expands to lunar South Pole/base operations, while keeping **visual fidelity** and **physics validity** as separate tracks until a gate allows them to converge.

The project should look compelling because of physically meaningful geometry, lighting, motion, scale, and telemetry—not because of decorative sci-fi UI.

Current truth:
- The reset-safe harness and known-answer calibration are working.
- R2 Test B fails both linear and angular momentum limits.
- G1 Test B passes the linear limit but narrowly fails the angular limit.
- Therefore neither R2 nor G1 is currently a canonical validated zero-g physics actor.
- R2 may be used now as a clearly labeled visual/kinematic actor.
- G1 may be used now as a candidate model in separately scoped experiments, but not described as zero-g validated.

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
4. **16–22 s — Engineering honesty:** compact calibration PASS / R2 FAIL / G1 FAIL card. No dramatic red warning screen.
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
- **G1 now:** independent candidate for controlled experiments and later lunar walking. Its zero-g Test B is still a FAIL.
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

## Claim ladder

### Allowed now
- “Built a reset-safe zero-g momentum validation harness in Isaac Sim.”
- “Known-answer calibration bodies pass the fixed conservation thresholds.”
- “R2 and G1 expose remaining model/solver residuals under the current Test B; I am keeping the failures visible rather than tuning the threshold.”
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

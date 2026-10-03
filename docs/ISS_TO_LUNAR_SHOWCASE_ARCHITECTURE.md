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

# Research Baseline

Snapshot date: 2026-09-30 (KST)

This is a project-scoped index, not the external evidence SSOT. The corresponding links were captured into PKI with provenance on 2026-09-30; local item/resource pointers are recorded in [PKI_CAPTURE_RECEIPT.md](PKI_CAPTURE_RECEIPT.md). Re-check live versions and licenses before implementation.

## 1. Core platform references

| Source | Role | Current use |
| --- | --- | --- |
| https://nvidia-omniverse.github.io/omniverse-labs/projects/astra-omniverse-libraries/ | Astra + Omniverse Libraries ISS example | OpenUSD ISS / telemetry architecture reference |
| https://github.com/NVIDIA-Omniverse/omniverse-labs | Omniverse Labs source | implementation provenance |
| https://github.com/isaac-sim/IsaacSim | Isaac Sim | physics/sensors/OpenUSD runtime |
| https://github.com/isaac-sim/IsaacLab | Isaac Lab | RL / policy training |
| https://github.com/OmniLRS/OmniLRS | lunar simulator on Isaac Sim | primary lunar environment reuse candidate |

NVIDIA's Astra ISS example states that publicly available NASA assets were assembled into an OpenUSD ISS model containing 555 exterior meshes and seven interior modules, then rendered/streamed with Omniverse libraries and ISS telemetry. This is evidence for asset/runtime architecture, not evidence that the published demo is a validated robotics physics environment.

OmniLRS is especially important for Phase 2 because it already uses NASA LRO DEM data, procedural rock/crater distributions, realistic illumination, ROS2 integration, and lunar mission-oriented telemetry models on the Isaac/Omniverse stack.

## 2. ISS / microgravity assets and software

| Source | Role | Caveat |
| --- | --- | --- |
| https://github.com/nasa/astrobee | ISS free-flyer software + simulator + ISS world | Gazebo/ROS reference; convert/reuse selectively |
| https://github.com/OnOrbitROS/Simulation | on-orbit robotics simulation framework | later EVA/orbital-dynamics coupling reference; outside ISS v0 |
| https://github.com/ihmcrobotics/valkyrie | Valkyrie software/control | candidate humanoid baseline, not automatically space-qualified |
| https://github.com/gkjohnson/nasa-urdf-robots | Robonaut 2 + Valkyrie URDF conversions | derivative/community repo; audit against original sources |
| https://github.com/Cheng-zhao916/Taikobot-urdf | Taikobot URDF candidate | provenance/inertia/license audit required |
| https://science.nasa.gov/3d-resources/ | NASA 3D models | visual source; collision/inertia must be rebuilt/verified |

## 3. ISS / microgravity papers

| Paper | Link | Project relevance |
| --- | --- | --- |
| APIARY ISS Astrobee testing | https://arxiv.org/abs/2512.03729 | Isaac Lab/PPO zero-g sim-to-space precedent |
| Whole-body motion without contacts and gravity | https://doi.org/10.1109/ICRA48506.2021.9560739 | centroidal-momentum baseline for floating humanoid attitude control |
| Free-floating two-arm humanoid trajectory optimization/control | https://doi.org/10.2514/1.G006828 | handrail / free-floating humanoid control baseline |
| RAMP reaction-aware microgravity locomotion | https://doi.org/10.1109/ICRA48891.2023.10161185 | contact reaction / locomotion objective design |
| Gait-Level Motion Design for Grasp-Based Dynamic Locomotion in Microgravity | https://doi.org/10.48550/arXiv.2605.21704 | recent grasp-to-grasp locomotion design/evaluation reference |
| SKYWALKER | https://doi.org/10.1109/CAI68641.2026.11536641 | microgravity analog crawling RL + sim-to-real evaluation reference |
| Taikobot | https://doi.org/10.3390/machines10100933 | full-size free-flying humanoid, push-flight-park strategy |

### ISS v0 hypotheses

H1. A floating humanoid can control body attitude through whole-body momentum redistribution without external contact; reproduce against a deterministic baseline before RL.

H2. Handrail locomotion can be expressed as a repeatable grasp → pull/push → flight/swing → regrasp → park task with reaction-aware metrics.

H3. Learned policies should be evaluated against controller/optimization baselines under identical mass, inertia, contact, latency, and target randomization.

### ISS v0 acceptance gates

1. Robot and environment each have a reviewed provenance record: source URL, license, upstream/derivative lineage, pinned version/hash, scale, frames, collision strategy, inertial-data source, and known uncertainty.
2. Zero-gravity physics passes both a no-actuation test and a prescribed internal-actuation test with no external contact. Reference frame, test duration, and numeric linear/angular-momentum tolerances must be fixed in config before the run is accepted.
3. Grasping suitability is demonstrated on two collision-proxy handrails with predeclared pass thresholds for target error, contact stability/slip, impulse/force, collision, and duration.
4. Metrics are logged independently of controller/policy implementation with units and success thresholds fixed before evaluation.
5. Visual ISS geometry is deferred until the physics slice passes; when introduced, visual geometry and collision proxies remain separate and traceable.

## 4. Lunar environment references

| Source | Role |
| --- | --- |
| https://github.com/OmniLRS/OmniLRS | Isaac-based lunar terrain / environment implementation |
| https://arxiv.org/abs/2309.08997 | OmniLRS paper |
| https://www.nasa.gov/moonbase-environment-visualization/ | NASA Moon Base / South Pole operating environment |
| https://www.nasa.gov/reference/moonbase-environment/ | South Pole lighting, terrain, dust, shadow/resource context |
| https://software.nasa.gov/software/MSC-27522-1 | NASA DUST: LRO DEM + SPICE + synthetic surface features |
| https://science.nasa.gov/3d-resources/ | NASA lunar/mission assets |

NASA's Moon Base material emphasizes the lunar South Pole rather than a generic equatorial plain: rugged slopes/craters, low-angle illumination, deep or persistent shadows, abrasive dust, and resource/operations constraints. Phase 2 should therefore preserve site/terrain provenance and lighting assumptions instead of using a purely decorative moon surface.

## 5. Lunar robotics papers

| Paper | Link | Project relevance |
| --- | --- | --- |
| Training Humanoid Robots to Walk in Lunar Gravity Using Reinforcement Learning (ACRA 2025) | https://www.australiancobotics.org/publication/training-humanoid-robots-to-walk-in-lunar-gravity-using-reinforcement-learning/ | Unitree G1 + Isaac Sim 5.0 direct lunar-humanoid baseline |
| Persistent multi-robot lunar-base construction | https://doi.org/10.1016/j.actaastro.2025.01.014 | later logistics/construction coordination layer |
| Vision+force DRL lunar construction assembly | https://doi.org/10.1016/j.actaastro.2025.01.017 | assembly / manipulation / sim-to-real reference |
| Constrained RL legged mobile manipulation for lunar operations | https://arxiv.org/abs/2510.12684 | safety-constrained mobile manipulation |

### Lunar v0 hypotheses

L1. Fine-tuning a validated Earth-gravity humanoid locomotion policy at 1.62 m/s² is a stronger baseline than training only from scratch; measure sample efficiency and gait change explicitly.

L2. Terrain/illumination randomization should be separated from robot-dynamics randomization so failure sources remain diagnosable.

L3. Lunar-base construction should not begin as a multi-robot fleet problem. Prove single-robot locomotion and energy metrics first, then add manipulation and later scheduling/task allocation.

### Lunar v0 acceptance gates

1. Lunar gravity magnitude, direction, world frame, physics step/substeps, solver settings, and contact material are pinned in config.
2. Walking suitability is demonstrated on rigid terrain before DEM/regolith integration; success, fall, slip, speed, tracking, torque, and energy thresholds are fixed before evaluation.
3. Earth-baseline, lunar fine-tune, and lunar-from-scratch comparisons use separate recorded training seeds and evaluation seeds.
4. South-Pole terrain/lighting integration starts only after the rigid-ground baseline passes, with site/DEM/date/illumination provenance recorded separately.

## 6. Shared ISS → Lunar experiment contract

Every accepted experiment must record at least:

- simulator + package versions and Git commits
- robot asset source/hash and validated inertial data source
- environment/site source/hash and coordinate frames
- gravity vector and any external-force model
- physics step/substeps and solver settings
- controller/policy identifier, training seeds, and separate evaluation seeds
- task start/goal/contact definitions and initial conditions
- metric names, units, energy definition, and success/failure thresholds fixed before evaluation
- success/failure reason
- pose/velocity error, momentum, contact forces/impulses, torque, energy, collisions, duration
- generated artifacts and verification receipt

## 7. First implementation slice

Do **not** start by building the full ISS or a full lunar base.

**Slice A — ISS physics v0:** one floating humanoid, zero-g, two simple collision-proxy handrails, one prescribed internal-motion conservation test, and one deterministic handrail transfer task. ISS module visuals are deferred.

**Slice B — Moon physics v0:** one rigid terrain patch, lunar gravity, one humanoid that has separately passed walking-suitability checks, and one deterministic waypoint locomotion task. South-Pole DEM, lighting, regolith deformation, base assets, and manipulation are deferred.

The same provenance, logging, artifact, and evaluation interfaces should run in both slices. Controllers and success predicates may remain task-specific. Differences in task physics belong in configuration, not duplicated framework code.

## 8. Open questions (explicitly UNKNOWN)

- Which humanoid model will be canonical: NVIDIA/IHMC Valkyrie, Unitree G1, Taikobot reconstruction, or another platform?
- Whether NASA/Astra ISS interior assets are directly reusable as simulation geometry or only as visual/reference assets after license/provenance review.
- Which exact lunar South Pole tile/site will be canonical.
- Whether OmniLRS current main branch or an Isaac Sim 6 migration branch is the best integration base.
- Whether regolith deformation/terramechanics is required for the first lunar experiment.
- Whether later EVA/on-orbit phases need a separate orbital-dynamics service or OnOrbitROS-derived model.

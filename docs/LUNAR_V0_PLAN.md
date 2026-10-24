# Lunar v0 Plan (Slice B: Moon physics v0)

Status: **gates APPROVED by the project owner on 2026-10-23 (section 5); implementation not started.** Downloads and long GPU training runs remain approval gates.

Sources of scope:
- [RESEARCH_BASELINE.md](RESEARCH_BASELINE.md): §5 lunar hypotheses and gates, §6 shared contract, §7 Slice B.
- [ISS_TO_LUNAR_SHOWCASE_ARCHITECTURE.md](ISS_TO_LUNAR_SHOWCASE_ARCHITECTURE.md): scene S5.
- AGENTS.md (lunar v0 = surface robotics at lunar gravity, no regolith model until one is needed and validated).

## 1. Goal and claim

**Goal.** One G1 walks a fixed waypoint course on a rigid patch at lunar gravity, with the same provenance, logging and metric contract as the ISS runs.

**Allowed claim after v0 passes:** "Evaluated lunar-gravity locomotion on a rigid patch with recorded success, fall, slip, tracking, torque and energy metrics."

**Not claimed:**
- regolith or terramechanics;
- South-Pole terrain or lighting;
- manipulation or base operations.

## 2. Scope

| In v0 | Deferred (in this order, each behind its own gate) |
| --- | --- |
| Flat rigid ground patch, 1.62 m/s² along −z | Lunar-gravity gait adaptation study (L1) |
| Unitree G1 `g1_29dof_rev_1_0` (the ISS canonical model, URDF hash already recorded) | South-Pole DEM + low-angle Sun (OmniLRS reuse candidate) |
| Waypoint course: straight 10 m, 90° turn, 2 m stop-and-go | Mobile manipulation |
| Earth-gravity baseline on the same patch (9.81 m/s²) | Regolith / deformable terrain (only with a validated model) |
| Video: ISS 0 g ↔ Moon 1/6 g split screen (S5 + end frame) | Multi-robot logistics / construction |

## 3. Reuse from the ISS phase

**Experiment record.**
- Today it covers scene/URDF/spec hashes, gate config version, gravity, control gains, per-step status, contacts, penetration and loads.
- The fields that `iss_activity_run.py` and `iss_space_demos.py` write become a shared record schema.
- The lunar runner fills in the same keys, plus gait metrics.

**Metric definitions.**
- External force = d(total linear momentum)/dt.
- Joint work = Σ|projected joint force × joint speed|.
- Contact-event counting.
- Momentum/COM utilities.

**Physics config pattern.** A pinned JSON config, the way `configs/iss_v0_momentum_v3.json` pins the ISS gate. New file: `configs/lunar_v0_rigid.json` (gravity, dt, solver, iterations, contact material).

**Rendering.**
- `render_iss_activity.py` replays any recorded run: link poses plus extras, chase or fixed camera, an overlay drawn from the record.
- `compose_gravity_comparison.py` makes the split screen.

**Detached runner pattern.** A GPU-free check before starting, so the user's other IsaacLab jobs are not disturbed.

## 4. Controller path

The zero-g hand-over-hand planner does not apply to walking. Lunar v0 needs a learned locomotion policy.

| Step | What | Where | Cost (estimate, UNKNOWN until measured) |
| --- | --- | --- | --- |
| P0 | Check for an existing G1 velocity checkpoint. None found in the local IsaacLab install's logs (only UR10e/Franka runs). | — | minutes |
| P1 | Earth baseline: train the IsaacLab `G1` flat velocity task (`isaaclab_tasks/.../locomotion/velocity/config/g1`, IsaacLab @418ca31) at 9.81 m/s². | local RTX 4070 Ti | several hours; GPU shared with the user's IsaacLab jobs |
| P2 | Lunar fine-tune from P1 at 1.62 m/s² (hypothesis L1). | same | ~1/3 of P1 (guess) |
| P3 | Lunar from scratch at 1.62 m/s² (L1 comparison). | same | ≈ P1 |
| P4 | Evaluate P1 at 9.81 and P2/P3 at 1.62 on the waypoint course in the project scene, with fixed evaluation seeds. | Isaac Sim 6.0.1 | < 1 h |

- **Seeds:** training seeds and evaluation seeds are recorded separately (gate 3).
- **Directly relevant prior work:** "Training Humanoid Robots to Walk in Lunar Gravity Using RL" (ACRA 2025) uses G1 in Isaac Sim 5.0. Read it before P1 to reuse reward and observation choices, and cite it.

## 5. Gates

Thresholds approved by the project owner on 2026-10-23, before any evaluation (gate 2). They are fixed for v0: changing them after results exist would be tuning to pass. They are criteria, not results.

| Metric | Definition | PASS (approved) |
| --- | --- | --- |
| Success | reaches every waypoint within 0.3 m, no fall | ≥ 9/10 evaluation seeds |
| Fall | pelvis height < 0.45 m, or torso tilt > 60° | 0 on PASS seeds |
| Slip | foot horizontal speed while in contact (contact sensor) | median < 0.05 m/s |
| Tracking | commanded vs measured base velocity, RMS | < 0.15 m/s |
| Torque | any joint at its URDF effort limit | < 5 % of steps |
| Energy | Σ\|τ·q̇\|·dt per metre (cost of transport also reported) | recorded; compared Earth vs Moon, no threshold in v0 |

Config gate (gate 1): gravity vector, dt, substeps, solver, iterations and contact material are pinned in `configs/lunar_v0_rigid.json`, and the runner refuses to run if the scene differs.

## 6. Work order once approved

1. `configs/lunar_v0_rigid.json` + `scripts/build_lunar_v0.py`.
   - The scene: a 20 × 20 m rigid patch, a stated lunar-regolith-grey material (visual only), a low-angle Sun (visual), the waypoint markers, and the same `/World/Actors`, `/World/Task` layout as the ISS scenes.
   - Checker in the style of `check_iss_full.py`.
2. P0/P1 training run, detached and GPU-aware; logs stay outside git, with the run ID recorded.
3. `scripts/lunar_walk_run.py`: evaluation runner writing the shared record (Earth and Moon, fixed seeds).
4. P2/P3, then evaluation.
5. Renders: S5 clip, ISS ↔ Moon split end frame.
6. Docs: a receipt in the architecture doc, mirroring the ISS receipts.

## 6a. Run log

- **2026-10-24: P1 queued.**
  - Task `Isaac-Velocity-Flat-G1-v0`, rsl_rl PPO, 1500 iterations (task default), training seed 42, Earth gravity (task default), run name `lunar_v0_p1_earth`.
  - Logs are in the local IsaacLab install, outside git.
- **Deviation from section 2:** the IsaacLab G1 tasks use IsaacLab's own G1 asset (`G1_MINIMAL_CFG`, Nucleus `Robots/Unitree/G1/g1_minimal.usd`), not the project's `g1_29dof_rev_1_0` URDF. v0 results apply to that asset. Moving the policy to the project URDF needs its own task config and is deferred.

## 7. Open items / UNKNOWN

- **Fall threshold for G1 at lunar gravity:** whether 0.45 m pelvis height suits G1 at 1/6 g is UNKNOWN (bounding gaits may be lower). Calibrate on the Earth baseline before fixing it.
- **G1 contact sensors in the URDF import:** whether they give foot-level contact forces is UNKNOWN; the slip metric depends on it.
- **OmniLRS on Isaac Sim 6.0.1:** compatibility is UNKNOWN (baseline open question §8). It is not needed for v0.
- **Unitree G1 asset license:** unitree_ros @ 5994d4f (assets/raw/unitree_ros_5994d4f/LICENSE) is BSD-3-Clause, Unitree Robotics. Not yet recorded in a project asset audit; whether it covers the meshes in rendered media is UNKNOWN until reviewed.
- **Training time:** UNKNOWN until P1 runs. GPU time is shared with the user's own IsaacLab jobs.

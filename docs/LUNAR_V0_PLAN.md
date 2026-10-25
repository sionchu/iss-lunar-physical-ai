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

## 6b. Results (2026-10-24): v0 gates NOT PASSED

**Training** (Isaac Lab `Isaac-Velocity-Flat-G1-v0`, rsl_rl PPO, training seed 42, logs in the local Isaac Lab install):

| Run | What | Gravity |
| --- | --- | --- |
| P1 | 1500 iterations | 9.81 |
| P2 | 500 iterations fine-tuned from P1's `model_1499` | 1.62, confirmed in the run's `params/env.yaml` |
| P3 | 1500 iterations from scratch | 1.62 |

**Evaluation** (`scripts/lunar_walk_eval.py`, seeds 1000–1009; outputs/lunar_v0/*_eval.json, gitignored). Medians are over successful seeds, against the gates approved in section 5:

| Policy @ gravity | Success (≥ 9/10) | Falls | Slip median (< 0.05 m/s) | Tracking RMS (< 0.15 m/s) | Torque-limit steps (< 5 %) | Energy J/m | v0 gate |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| P1 Earth @ 9.81 | **10** | 0 | **0.009** | **0.130** | 39 % | 272 | FAIL (torque) |
| P1 Earth @ 1.62 (zero-shot) | 2 | 8 seeds fell | 0.077 | 0.363 | 5.2 % | 264 | FAIL |
| P2 lunar fine-tune @ 1.62 | **10** | 0 | 0.084 | 0.313 | **0 %** | 174 | FAIL (slip, tracking) |
| P3 lunar scratch @ 1.62 | **10** | 0 | **0.038** | 0.244 | **1.1 %** | 152 | FAIL (tracking) |

**Observations** (diagnostic, not gate changes):
- **Torque:** P1's torque-limit steps are almost all the ankle pitch joints at their 20 N·m limit (right 20 %, left 18 % of steps).
- **Earth policy on the Moon:** it falls without adaptation, mostly at a pelvis height just under 0.45 m, around 20 s.
- **Energy:** both lunar policies finish every seed and use 36–44 % less energy per metre than P1 on Earth.
- **Tracking:** the error is measured against the waypoint follower's commands, which include stops and turns. The gates were fixed before evaluation and are not changed.
- **L1 (fine-tune vs scratch):** after 500 fine-tune iterations, fine-tuning did not beat training from scratch on slip or tracking. Only one training seed per arm was run, so this is not a conclusion.

**Next (policy work only; the gates stay fixed):**
- train longer or with more weight on velocity tracking at 1.62 m/s²;
- add training seeds;
- re-evaluate on the same 10 evaluation seeds.

### P4 retraining (2026-10-24/25): still NOT PASSED

**Setup:** from scratch at 1.62 m/s², 3000 iterations, velocity-tracking reward weight 1.0 → 2.0. The override is confirmed in each run's `params/env.yaml`. Training seeds 42 and 43. Same course, same 10 evaluation seeds, same gates.

| Policy @ 1.62 | Success | Slip median | Tracking RMS | Torque-limit steps | Energy J/m | v0 gate |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| P3 (reference) | 10 | 0.038 | 0.244 | 1.1 % | 152 | FAIL (tracking) |
| P4, training seed 42 | 10 | 0.067 | 0.235 | 65 % | 206 | FAIL (slip, tracking, torque) |
| P4, training seed 43 | 10 | 0.075 | 0.243 | 97 % | 186 | FAIL (slip, tracking, torque) |

- **Tracking reward:** doubling its weight did not lower the tracking error (0.235–0.243 vs 0.244). It traded away slip and torque instead: the ankles now sit at their limit most of the time.
- **Tracking floor:** every lunar-gravity policy so far has its per-seed tracking error between 0.22 and 0.28 m/s, while the Earth policy on Earth gets 0.13.
- **Likely cause:** the instantaneous base velocity oscillating within each low-gravity stride, which an instantaneous RMS counts as error. This is UNKNOWN until velocity time series are recorded.
- The gate stays as approved. Any stride-averaged metric would be an extra, separately reported diagnostic, added only with the owner's agreement.

### Tracking diagnosis (2026-10-25)

**Method:**
- `lunar_walk_eval.py --timeseries` re-ran P1 at 9.81 and P3 at 1.62 on evaluation seeds 1000 and 1001 (outputs/lunar_v0/diag_*_ts.json).
- `scripts/lunar_tracking_diag.py` splits the gate's RMS into:
  - steady walking: command at least 0.75 m/s, no turning, for 1.5 s;
  - transitions: stops, starts and turns.
- Within steady walking it further separates the slow error (1 s average) from the within-stride oscillation.

| Run | Gate RMS | Steady RMS | Transition RMS | Steady slow error | Within-stride oscillation (fwd / lat) | Stride period |
| --- | ---: | ---: | ---: | ---: | --- | ---: |
| P1 Earth @ 9.81 | 0.12–0.13 | 0.06 | 0.16–0.17 | 0.05 | 0.014 / 0.028 | 0.46 s |
| P3 Moon @ 1.62 | 0.23–0.24 | 0.065 | 0.30–0.32 | 0.06 | 0.020 / 0.017 | 0.36 s |

- **Hypothesis rejected:** within-stride oscillation is small (≤ 0.03 m/s) on both bodies, so it is not what drives the error.
- **Steady walking is the same on Earth and Moon:** RMS about 0.06, about 0.05 m/s slower than commanded.
- **The lunar excess is entirely in transitions:** 0.30–0.32 vs 0.16–0.17. The waypoint follower changes the command as a step (for example 0.8 → 0 m/s at the stop), and the lunar policy takes longer to reach it.
- **Physical explanation (consistent but not shown):** traction-limited horizontal acceleration scales with gravity, about μ·g, so changes in speed take longer at 1.62 m/s².
- **Conclusion:** the tracking gate is measuring the evaluation's step command profile at lunar gravity, not gait quality.
- The gate is unchanged. Changing the command profile (for example, acceleration-limited commands applied identically on Earth and the Moon) would change the evaluation protocol, and needs the owner's approval before any re-evaluation.

## 7. Open items / UNKNOWN

- **Fall threshold for G1 at lunar gravity:** whether 0.45 m pelvis height suits G1 at 1/6 g is UNKNOWN (bounding gaits may be lower). Calibrate on the Earth baseline before fixing it.
- **G1 contact sensors in the URDF import:** whether they give foot-level contact forces is UNKNOWN; the slip metric depends on it.
- **OmniLRS on Isaac Sim 6.0.1:** compatibility is UNKNOWN (baseline open question §8). It is not needed for v0.
- **Unitree G1 asset license:** unitree_ros @ 5994d4f (assets/raw/unitree_ros_5994d4f/LICENSE) is BSD-3-Clause, Unitree Robotics. Not yet recorded in a project asset audit; whether it covers the meshes in rendered media is UNKNOWN until reviewed.
- **Training time:** UNKNOWN until P1 runs. GPU time is shared with the user's own IsaacLab jobs.

## 8. After v0: footprints and dust (approved direction, not started)

**Footprints: one-way terramechanics.**
- The robot still walks on rigid physics.
- At each foot contact, sinkage z comes from the Bekker pressure-sinkage relation p = (k_c/b + k_phi)·z^n, using the measured foot contact force and contact width b.
- A terrain heightfield is stamped with that depth.
- **Claim limit:** load-dependent footprints from cited soil parameters. This is not a validation of regolith physics.
- **Candidate parameter sources** (each to be read and cited before use; no value is adopted yet):
  - **Geotechnical ranges:** friction angle about 30–50°, cohesion about 0.1–1 kPa at the surface ("Lunar Dust: Formation, Microphysics, and Transport", https://arxiv.org/pdf/2511.08503, citing Apollo, Lunokhod and analog data).
  - **Bekker fits for a lunar simulant:** n 0.70–1.12; k_c and k_phi depend on bulk density ("Study on the Pressure-Sinkage Relationship of Lunar Soil Simulant", https://www.scientific.net/AMR.508.241).
  - **Effect of low gravity:** lower n, k_phi and c; no clear change in k_c or φ (Journal of Terramechanics 2015, https://www.istvs.org/publication-news/2015/6/5/effect-of-gravity-on-the-mechanical-properties-of-lunar-regolith-tested-using-a-low-gravity-simulation-device).
  - **Recent pressure-sinkage tests:** NASA NTRS 20250003024 (https://ntrs.nasa.gov/api/citations/20250003024/downloads/SRR%202025%20Pressure%20Sinkage%20Testing%20FINAL%20Mueller.pdf).

**Dust: ballistic ejecta.**
- In vacuum at 1.62 m/s², grains kicked up at foot strike and lift-off follow drag-free parabolas.
- Simulated as particles with a stated launch-velocity distribution, then rendered.
- Electrostatic levitation or adhesion has no validated model here; it is not modelled, or only as a labelled illustration.

# R2 Test B angular-momentum diagnosis

Date: 2026-09-30. This is a diagnostic study of the existing R2 derivative, not a gate rerun or a replacement verdict. The official `configs/iss_v0_momentum.json`, raw assets, and [gate record](ISS_V0_MOMENTUM_GATE.md) were left unchanged. Isaac Sim 6.0.1 ran through `C:\isaacsim\python.bat`. Outputs and full Isaac logs are under ignored `outputs/diagnostics/`; the scripts and variant configs are tracked.

## 2026-10-04 reset-safe supersession

The sequence sensitivity identified by this study was subsequently confirmed as a harness defect: Test B inherited Test A's final joint positions. The current gate now reopens a fresh articulation state for every test and proves A→B / B-only sequence independence by initial-state hash and identical result ratios. Under that reset-safe lifecycle, R2 Test B is **FAIL** with linear ratio **0.002085838** and angular ratio **0.028239941**. See [ISS_V0_MOMENTUM_GATE.md](ISS_V0_MOMENTUM_GATE.md).

Accordingly, every ablation value below—including V9's historical 0.007721 PASS—belongs to the obsolete sequence-matched A→B initialization. The study remains useful for locating numerical sensitivities and motivating asset/inertia investigation, but its variant verdicts are **not current gate evidence**. V9 must not be carried into the official config, and any future use of these ablations requires rerunning them under the reset-safe lifecycle.

## Reproduction and measurement

The then-current `scripts/iss_v0_momentum_test.py` at diagnostic commit `0719cca`, with `outputs/r2_derived/r2c6_zero_g.urdf`, reproduced Test A **PASS** (linear ratio 0.000067002; angular ratio 0.002325688) and the sequence-dependent Test B **FAIL** (linear ratio **0.000041366**; angular ratio **0.013671235**, limit 0.01). Its structured result is `outputs/diagnostics/baseline_repro.json`. The instrumented A-then-B baseline reproduced the same ratios exactly. Total angular momentum is measured about the instantaneous system COM using the corrected PhysX tensor interpretation in `scripts/momentum_measure.py`.

The old harness's B result depended on running A first: a fresh B-only diagnostic run gave angular ratio 0.028239941 and linear ratio 0.002085838, while the A-then-B sequence gave 0.013671235 and 0.000041366. Therefore every ablation in this study replayed the 10 s A sequence to preserve that historical B initialization, then measured B with one factor changed. A passed in these runs, but only B was used to rank the historical ablations. At the time the exact mechanism was UNKNOWN; the 2026-10-04 reset work confirmed that B inherited A's final joint positions. The fresh B-only values above are now reproduced exactly by the reset-safe official lifecycle.

## Per-step findings at the matched baseline

`outputs/diagnostics/baseline_matched_trace.json` contains all 2,401 B samples with `L(t)` and vector `delta_L` per step, plus per-group angular momentum about system COM. The total starts at zero and ends at approximately `(-0.08278, -0.48984, 0.30340)` kg m²/s. Its magnitude reaches 0.58210 kg m²/s at 10 s. Median `|delta_L|` is 0.001548 kg m²/s per step, 95th percentile 0.005174, and the largest step is 0.064352 at 8.395833 s. This is an accumulating, irregular drift with spikes, not a pure periodic oscillation. Detrended components contain 0.5 Hz and 1 Hz content from the commanded motion, as well as slower 0.1–0.2 Hz content; no single actuation harmonic explains the accumulated error.

The table reports the peak norm of each **net group** `L_i(t)` and peak per-link speeds within the group. Group momenta can be much larger than total momentum because they cancel. Names were classified from imported link names; torso/waist/head is the residual group.

| Link group | Peak `|L_i|` kg m²/s | Peak link linear m/s | Peak link angular rad/s |
| --- | ---: | ---: | ---: |
| Left arm | 12.3640 | 1.9916 | 16.3602 |
| Right arm | 11.9144 | 2.1102 | 17.9973 |
| Right hand | 3.8525 | 3.0230 | 29.2531 |
| Torso/waist/head | 3.7324 | 0.9875 | 4.3196 |
| Left leg | 2.6425 | 0.3118 | 1.7395 |
| Right leg | 2.1709 | 0.3255 | 1.3729 |
| Left hand | 0.2306 | 7.3217 | 131.1064 |
| Left leg gripper | 0.0161 | 0.3423 | 1.3061 |
| Right leg gripper | 0.0089 | 0.2535 | 1.3279 |

At the largest total-L step, the right-hand group changes by `|delta L_i|=1.1078` and the right arm by 0.7943 kg m²/s; large opposing internal transfers leave a 0.06435 kg m²/s total residual. The zero-gain left index distal joint peaks at **195.77 rad/s** and is within 0.001 rad of a limit in **1,039 of 2,401** samples. Other zero-gain left finger joints reach 93–141 rad/s and hit limits hundreds of times. Thus passive fingers are numerically active, while leg-gripper links have small `|L_i|` and modest speed in this trace. Limit-hit counts use the PhysX joint limits and a 0.001 rad proximity threshold; they are a diagnostic indicator, not contact impulses.

PhysX link masses span **0.001000–47.770844 kg**. Principal link inertias span **4.6764e-8–1.995793 kg m²**. For each driven arm joint, the table compares its child link with the lightest *descendant* link (not merely its immediate child), using authored derivative URDF mass and principal inertia. This is a conditioning indicator, not proof of an erroneous physical value.

| Driven joint, each side | Child link | Child mass kg | Lightest descendant | Descendant mass kg | Mass ratio | Smallest-principal-inertia ratio, left/right |
| --- | --- | ---: | --- | ---: | ---: | ---: |
| joint0 | shoulder_roll | 7.070314 | left_thumb_proximal / right_middle_base | 0.001 | 7,070 | 209,345 / 64,199 |
| joint1 | shoulder_pitch | 4.727539 | same respective descendant | 0.001 | 4,728 | 113,815 / 34,904 |
| joint2 | upper_arm | 4.902571 | same respective descendant | 0.001 | 4,903 | 131,674 / 40,380 |
| joint3 | elbow | 2.337701 | same respective descendant | 0.001 | 2,338 | 41,837 / 12,830 |

## Single-factor ablations

Every JSON under `configs/diagnostics/` is a full copy of the gate config with a `diagnostic.single_factor_vs_gate` statement. V6/V7 additionally use isolated URDF copies generated under `outputs/diagnostics/`. All completed A sequences passed. `delta L` is B angular-ratio difference from the 0.013671235 baseline; negative is improvement. B angular and linear limits remain 0.01 and 0.001, respectively.

| Variant | Sole factor changed | B angular ratio | B linear ratio | B status | A | delta L |
| --- | --- | ---: | ---: | --- | --- | ---: |
| Baseline | Gate config and R2 derivative | 0.013671 | 0.0000414 | FAIL | PASS | 0 |
| V1 | dt 1/480 s, substeps 1 | 0.026159 | 0.0000734 | FAIL | PASS | +0.012488 |
| V2 | dt 1/960 s, substeps 1 | 0.024513 | 0.0001657 | FAIL | PASS | +0.010842 |
| V3 | Position iterations 64; velocity 4 | 0.024090 | 0.0000491 | FAIL | PASS | +0.010418 |
| V4 | Velocity iterations 16 | 0.015039 | 0.0000406 | FAIL | PASS | +0.001368 |
| V5 | PGS instead of TGS | 0.021121 | 0.0000392 | FAIL | PASS | +0.007450 |
| V6 | Lock 36 movable hand/finger joints | 0.022460 | 0.0000371 | FAIL | PASS | +0.008788 |
| V7 | Lock six leg-gripper joints | 0.016894 | 0.0000240 | FAIL | PASS | +0.003222 |
| V8 | Visual-mesh bbox inertia for 11 fallback spheres | NOT_RUN | NOT_RUN | NOT_RUN | NOT_RUN | — |
| V9 | 0.01 kg m² armature on zero-gain joints | 0.007721 | 0.0000440 | PASS | PASS | -0.005950 |
| V10 | Driven-arm PD gains 100/10 → 50/5 | 0.013918 | 0.0041016 | FAIL | PASS | +0.000247 |

V8 is **NOT_RUN** because all 11 `fallback_sphere` links have zero `<visual>` elements in the derivative URDF. There is no parseable visual mesh from which to compute the requested bbox. No substitute geometry was invented. V10 additionally fails the linear limit. The exit status of the Isaac batch wrapper is not used as a verdict; values above come from result JSON files.

V9 was repeated after adding an armature readback check. It reproduced the same B ratios, and PhysX reported 0.01 kg m² (float32 readback 0.00999999978) on all **66** zero-gain DOFs. The confirmation is `outputs/diagnostics/v9_zero_gain_armature_confirm.json`.

## Ranked explanation and owner options

1. **Historical evidence of sensitivity: numerical conditioning of the high-ratio, zero-gain joint tree under driven arm motion.** The 0.001 kg downstream links and 4.68e-8 kg m² minimum inertia coexist with heavy, rapidly moving arm links. Under the obsolete sequence-matched initialization, V9 alone lowered B angular ratio below the limit while preserving a passing A. This shows sensitivity to joint reflected inertia in that historical state; it neither proves the current reset-safe cause nor establishes that 0.01 kg m² is physically correct.
2. **Historical/suggestive: passive hand-joint motion and limit interactions may amplify the conditioning problem.** The high velocities and many limit hits are measured. However V6 failed even in the historical setup, so fingers alone were not an isolated explanation; fixing joints also changes imported articulation topology and internal inertia aggregation. Reset-safe confirmation remains NOT_RUN.
3. **Not standalone fixes in the historical settings:** smaller dt, more position or velocity iterations, PGS, locking only fingers, locking only leg grippers, and halving driven-arm PD gains. The already documented `world_ref` removal also left the historical gate result unchanged. These are not reset-safe ablation verdicts and do not prove irrelevance in combinations or other regimes.

Owner decisions, without changing the gate in this study:

- **(a) Physically justified asset correction:** obtain source CAD or measured mass, COM, joint rotor inertia and link inertia for light hand descendants and gripper parts; revise the derivative with provenance and independently validate the corrected inertial tensors. This is the preferred route to a physical model.
- **(b) Simulation-configuration change requiring a new versioned gate config:** after validating actual actuator reflected inertias, set documented joint armatures and repeat A/B plus independent conservation calibration under a new config/version. Solver or timestep choices also require a versioned gate and fresh evidence if changed.
- **(c) Not acceptable, tuning to pass:** copy V9's arbitrary 0.01 kg m² into the official gate without physical support, relax the 0.01 threshold, or delete/lock articulated parts solely to obtain a PASS.

## UNKNOWN and reproduction

UNKNOWN: original R2 mass/inertia and rotor-inertia provenance; the exact zero-gain joint subset responsible for the reset-safe residual; whether limit impulses or solver constraint impulses account for that residual; collision/mesh fidelity; and an independently validated inertia for the 11 fallback spheres. The A-to-B sequence-state mechanism itself is no longer UNKNOWN: B inherited A's final joint positions in the old harness. The ablation comparison is one deterministic historical run per factor, not a robustness distribution and not a reset-safe ranking. The diagnosis does not accept R2 for the official ISS v0 gate.

Historical reproduction only: run `python -m py_compile scripts/iss_v0_diagnose_r2.py scripts/make_r2_diagnostic_urdf.py`, generate V6/V7 with `python scripts/make_r2_diagnostic_urdf.py <variant>`, then run `C:\isaacsim\python.bat scripts/iss_v0_diagnose_r2.py --config configs/diagnostics/<variant>.json --urdf <derivative-or-diagnostic-URDF> --out outputs/diagnostics/<variant>.json`. Omit `--test-b-only` to preserve the matched A-then-B initialization. Add `--trace` for per-step B telemetry. This intentionally reproduces the obsolete sequence-matched diagnostic state and is not the current gate. Current reset-safe reproduction, including `--test-b-only`, is defined in [ISS_V0_MOMENTUM_GATE.md](ISS_V0_MOMENTUM_GATE.md).

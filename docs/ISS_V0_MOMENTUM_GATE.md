# ISS v0 zero-g momentum gate

Run date: 2026-09-30. Asset: scripted R2 derivative in `outputs/r2_derived/r2c6_zero_g.urdf`. Config was written before the runtime attempts: `configs/iss_v0_momentum.json`. The gate **FAILS**; R2 is not accepted as the ISS v0 physics model by this experiment.

## Fixed configuration and measurement

Isaac Sim 6.0.1, headless URDF importer, `fix_base=false`, zero gravity, 1/240 s physics step, one step per sample, 10 s per test, PhysX TGS, 16 position and 4 velocity iterations, zero rigid-body linear/angular damping, sleep threshold zero, no self collision, and zero joint friction. Test A has zero drive gains and an initial base twist of `(0.05, 0, 0)` m/s and `(0, 0, 0.1)` rad/s. Test B starts at rest and commands both arms' shoulder joints 0–2 and elbow joint 3 with a 0.5 rad, 0.5 Hz sine using position PD gains 100/10; all other drives have zero gains.

At every physics step, `scripts/iss_v0_momentum_test.py` reads link poses, velocities, masses, center-of-mass offsets/orientations and inertia tensors from Isaac's PhysX-backed tensor views. It computes world-frame `P = Σ mᵢ vᵢ` and `L = Σ [mᵢ(rᵢ−c)×vᵢ + Rᵢ Iᵢ Rᵢᵀ ωᵢ]` about the system COM. The code treats the tensor inertia as link-body-frame inertia about the link COM and rotates it by the link and COM orientations. It converts link-origin linear velocity to COM velocity using `ω×r_COM`. This tensor/frame interpretation is an implementation assumption pending independent validation against a simple known rigid body.

## Result

| Test | Max linear error | Linear limit | Max angular error | Angular limit | Verdict |
| --- | ---: | ---: | ---: | ---: | --- |
| A: no actuation | 10.931549 kg·m/s / 10.931917 = **0.999966** | 0.001 | 3.230971 kg·m²/s / 3.230931 = **1.000013** | 0.01 | **FAIL** |
| B: internal actuation | 17.638409 kg·m/s / `Pref` 89.208824 = **0.197720** | 0.001 | 4.106226 kg·m²/s / `Lref` 51.974198 = **0.079005** | 0.01 | **FAIL** |

`Pref = maxₜ Σ mᵢ|vᵢ|`; `Lref = maxₜ Σ |Lᵢ about system COM|`. Both tests sampled 2,401 states over 2,400 physics steps. Test A's measured total momentum changed from `P=(10.925553, −0.372968, −0.000777)` to `(0.000326, −0.001239, −0.000776)` kg·m/s and from `L=(0.044221, 0.025266, 3.230529)` to approximately zero kg·m²/s. Test B started with zero total momentum and ended with `P=(−3.801339, 1.061951, 10.767527)` kg·m/s and `L=(−0.961855, −2.048705, 1.089512)` kg·m²/s.

The A drift is dominated by loss of the initial whole-body momentum; at the largest angular deviation, the largest individual angular-momentum contribution was the left leg foot. In B it was the left lower arm. An unintended world constraint from the source URDF's massless `r2/world_ref` root and fixed `world_ref→robot_world` joint is a plausible cause, despite `fix_base=false`; it is not yet confirmed. The velocity/COM tensor interpretation also needs an independent check. The gate was not weakened and the asset was not altered in response to the failures.

## Reproduce

```powershell
python scripts/make_r2_zero_g_derivative.py
python -m py_compile scripts/make_r2_zero_g_derivative.py scripts/iss_v0_momentum_test.py
& C:\isaacsim\python.bat scripts/iss_v0_momentum_test.py *> outputs/iss_v0_momentum.log
Get-Content outputs/iss_v0_momentum_result.json
```

The structured result is `outputs/iss_v0_momentum_result.json`; the complete runtime log is `outputs/iss_v0_momentum.log`. The Isaac wrapper returned exit code 0 even on a failing gate, so use the JSON test verdicts as the gate record. The first two startup/debug attempts stopped before both tests ran due to an unsupported scene time-step API and an unauthored PhysX articulation schema attribute; the final attempt ran both tests. The log also contains cache permission warnings.

## UNKNOWN

- The precise physical/numerical cause of the reset-safe R2 Test B linear and angular residuals.
- The precise cause of G1's reset-safe Test B angular residual (0.01134361), which is above the unchanged 0.01 limit.
- Source mass, COM, inertia and collision fidelity for the R2 derivative, including the 11 sphere fallbacks and the existing neck tensor flag.
- Physical fidelity of the local G1 URDF inertias/collisions for this zero-g use case; seven links have missing or identity-style inertias.
- Upstream R2 file-level NASA-1.3 license confirmation.
- Robustness across alternative internal motions, durations and repeated stochastic/nondeterministic runs; the reset check below proves sequence independence for this deterministic harness, not general model validity.

## Harness validation and reset-safe rerun — 2026-09-30 / 2026-10-04

This section supersedes the momentum *measurements* above; it does not alter `configs/iss_v0_momentum.json` or erase the historical runs. The same 10 s, 240 Hz, zero-g settings and predeclared limits were used throughout. A ratios are maximum deviation divided by initial total momentum (limits: P ≤ 0.001, L ≤ 0.01). B ratios are maximum total momentum divided by `Pref` and `Lref` (same limits).

A later diagnosis found that the old A→B harness did not restore joint positions before B. The old R2 B ratio 0.013671 and old G1 B ratio 0.005641 were therefore sequence-dependent and are **not valid current gate measurements**. On 2026-10-04 the harness was changed so every test reopens the imported USD and starts from the same fresh articulation state. The official config, tolerances, assets, inertias, gains and armatures were unchanged.

| Body / derivative | Test A P ratio | Test A L ratio | A | Reset-safe Test B P ratio | Reset-safe Test B L ratio | B |
| --- | ---: | ---: | --- | ---: | ---: | --- |
| Calibration free box | 0 | 0.000819 | **PASS** | — | — | **NOT_RUN**: no internal joint |
| Calibration free two-link | 0.00000422 | 0.001562 | **PASS** | — | — | **NOT_RUN**: zero-drive calibration only |
| R2, `world_ref` kept | 0.0000670 | 0.002326 | **PASS** | **0.00208584** | **0.02823994** | **FAIL**: linear + angular |
| Unitree G1 29-DOF | 0.0000746 | 0.004488 | **PASS** | 0.000004662 | **0.01134361** | **FAIL**: angular |

The earlier `world_ref`-removed R2 comparison remains useful only for the anchor diagnosis: under the old sequence-dependent harness it matched the kept-`world_ref` result exactly. It was not rerun as a reset-safe candidate because that comparison had already ruled out the suspected world anchor and does not change the current asset-validation gate.

### Reset isolation evidence

For each robot, the A→B run and a separate `--test-b-only` run produced exactly the same B initial-state SHA-256, identical DOF positions (maximum difference 0), and identical P/L error ratios (difference 0 in the serialized results). Within each A→B run, A and B also start from the same recorded fresh-state hash before test-specific initial conditions are applied.

- R2 fresh-state hash: `7d94109cf49e3ef1f63c310eb28a9b7cadfada6586a2716f82010cb5e61a0805`
- G1 fresh-state hash: `005db1566d15bc1c8de39d9feebf71d7a6894a2919cad0c324f08315e1e60c95`
- R2 URDF SHA-256: `282152eee0c5fbf514f3cdf74b8ce842f659f32c634c7b9c147a7c0972a97d8f`
- G1 URDF SHA-256: `e1dc89366bf96aa33ba4ef211884f6d81320ddd2ac6f8b1ad82986e64008be31`

Reset-safe evidence is under ignored `outputs/reset/`: `r2_ab.json`, `r2_b.json`, `g1_ab.json`, `g1_b.json`, and their logs. The harness now records the complete pre-condition DOF position/velocity vectors plus a float32 state digest.

The calibration used a 2 kg free box (0.4 × 0.2 × 0.1 m), an authored COM offset of (0.05, 0.02, 0) m and a principal-axis rotation of 45° about X, plus two such boxes joined by a free, zero-drive Z revolute joint. Both received the Test A initial base twist. For the box, measured initial `P=(0.1000000015, 0, 0)` kg·m/s matches the 0.1 kg·m/s known answer; its initial `L=(0, -0.0002500, 0.0030833)` kg·m²/s. The two-link initial `P=(0.200000003, 0.03199999, 0)` includes transverse motion from the joint geometry. Both conserve P and L within the fixed limits over 2,400 steps. Their evidence is `outputs/iss_v0_calibration_result.json` and `outputs/iss_v0_calibration_final_result.log`.

Three earlier harness faults caused misleading original results. First, a tensor-set articulation twist was cleared on the first explicit `SimulationManager.step`; a one-step warmup **before** setting initial conditions fixes this, as seen in the two-link first-step probe. Second, `RigidPrim.get_velocities()` already returns COM linear velocity, so adding `ω × COM offset` double-counted transport. Third, the tensor inertia already includes principal-axis orientation in the link body frame; rotating it by the COM quaternion a second time produced false angular drift. The shared `scripts/momentum_measure.py` now uses COM velocity directly and rotates tensor inertia by link world orientation only. The offset-COM and rotated-axis calibration isolated these frame bugs. A fourth fault remained in test lifecycle: B inherited A's final joint positions. The 2026-10-04 fresh-stage reset removes that dependency and is the current gate basis; earlier B ratios are historical.

### Anchor diagnosis and independent control

The original URDF has inertial-free `r2/world_ref` joined to `r2/robot_world` by fixed `r2/fixed/world_ref/robot_world`. In the imported R2 USD, `/r2/Geometry/tn__r2world_ref_dH` is a parent Xform with no rigid-body, mass or inertia schema. Its child `/r2/Geometry/tn__r2world_ref_dH/tn__r2robot_world_eK` carries the articulation root, with mass 0.01 kg and diagonal inertia (0.0001, 0.0001, 0.0001) kg·m². The fixed `world_ref` joint is collapsed by import. No imported joint has an empty/world body target. With `--remove-world-ref`, the articulation root moves to `/r2/Geometry/tn__r2robot_world_eK`; both R2 momentum outcomes are numerically identical. Thus the suspected `world_ref` world anchor is not supported by the imported USD or the comparison.

The G1 control used the locally present official [`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros) checkout at commit `ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee`, file `robots/g1_description/g1_29dof.urdf`, SHA-256 `e1dc89366bf96aa33ba4ef211884f6d81320ddd2ac6f8b1ad82986e64008be31`. The checkout declares BSD-3-Clause. Its URDF has 40 links, 39 joints and 35.115142 kg authored mass; seven links have missing or identity-style inertias. Numeric URDF dimensions are interpreted as metres, masses as kg, and the imported USD is Z-up. It uses authored mesh and primitive collision geometry with self-collision disabled for this gate; source CAD accuracy, collision fidelity, COM/inertia measurements and actual contact behavior remain UNKNOWN. The pelvis articulation root has no world-side joint. Test B mapped each side's `shoulder_pitch`, `shoulder_roll`, `shoulder_yaw` and `elbow` joint to the same four-joints-per-arm command and config gains; all eight names matched. No new raw asset was downloaded or added to Git.

**Conclusion:** the original near-100% Test A loss was a harness initialization fault, with two additional tensor-frame calculation faults. A fourth harness defect then remained: Test B inherited Test A's final joint positions. After making every test start from a freshly reopened articulation state, both calibration bodies, R2 and G1 pass Test A, but **neither candidate passes Test B**. R2 fails both B limits (P 0.00208584, L 0.02823994); G1 passes B linear momentum but misses the angular limit narrowly (L 0.01134361 vs 0.01). The exact residual causes are UNKNOWN. Neither R2 nor G1 is accepted as the canonical ISS v0 physics asset by this gate. Do not relax thresholds or carry V9's unsupported armature into the official config to manufacture a PASS.

Reproduce the current gate with `C:\isaacsim\python.bat scripts/iss_v0_calibration.py`, then run `C:\isaacsim\python.bat scripts/iss_v0_momentum_test.py --urdf <URDF> --out <AB.json>` and `C:\isaacsim\python.bat scripts/iss_v0_momentum_test.py --urdf <URDF> --test-b-only --out <B.json>`. Compare the two B `initial_state.sha256_float32` values, DOF positions and error ratios before accepting sequence independence. R2 uses `outputs/r2_derived/r2c6_zero_g.urdf`; G1 resolves from the local Unitree checkout above. The 2026-10-04 reset-safe records are `outputs/reset/r2_ab.json`, `r2_b.json`, `g1_ab.json`, and `g1_b.json`. The Isaac wrapper may return exit code 0 even when the JSON gate verdict is FAIL; the structured JSON verdict is authoritative.

## v2 vendor-armature check (2026-10-04)

**Verdict: FAIL for both candidates. Joint armature does not close Test B; the v1 gate and its verdicts are unchanged.**

`configs/iss_v0_momentum_v2_armature.json` (SHA-256 `867903f35b8133be6000432ad3614a1ba35a97decf026fd5b375325795bbba59`) is v1 plus `physics.joint_armature_kg_m2 = 0.01` on every DOF. Provenance: Unitree's vendor MuJoCo model `unitreerobotics/unitree_mujoco` `unitree_robots/g1/g1_29dof.xml` sets `armature="0.01"` on all motor classes (same value in `google-deepmind/mujoco_menagerie` `unitree_g1/g1.xml`). It is a vendor model value, not derived from rotor inertia × gear ratio². For R2 the value has no provenance, so R2 rows are diagnostic. `configs/diagnostics/v2d_undriven_armature.json` (diagnostic only) applies it to undriven DOFs, the reset-safe analogue of the old V9. `scripts/iss_v0_momentum_test.py` gained `--config`; armature is applied after the fresh-stage reset and read back, and every result records `config_version`, `config_sha256` and the per-test DOF armature range. Results are in `outputs/v2_armature/` (gitignored).

| Run | Config | Test A (P / L) | Test B P ratio (≤0.001) | Test B L ratio (≤0.01) | B |
| --- | --- | --- | --- | --- | --- |
| G1 baseline re-run | v1 | PASS | 0.00000466 | 0.01134361 | FAIL (reproduces the reset-safe record) |
| G1 | v2, all 29 DOFs | PASS | 0.00000248 | 0.02029583 | FAIL (worse) |
| G1 | v2d, 21 undriven DOFs | — | 0.00000294 | 0.01528290 | FAIL |
| R2 | v2, all 74 DOFs (diagnostic) | PASS | 0.00002694 | 0.01336100 | FAIL |
| R2 | v2d, 66 undriven DOFs (diagnostic) | — | 0.00002357 | 0.01262060 | FAIL |

- B-only runs reproduced the A→B values exactly (same initial-state hash per robot: G1 `005db1566d15…`, R2 `7d94109cf49e…`), and a re-run after reordering the armature code gave identical G1 v2 ratios.
- Armature lowers R2's linear residual below the limit (0.00209 → ~0.00002) and roughly halves its angular residual (0.0282 → 0.0126–0.0134), but neither run passes. The old V9 PASS came from the sequence-dependent harness and is not reproduced under the reset-safe lifecycle.
- For G1, vendor armature makes the angular residual worse; armature is not the cause of G1's Test B residual.
- Root cause of the remaining angular residuals: UNKNOWN. Next diagnostic candidates (not run): timestep / solver-iteration ablation under the reset-safe harness, and an independent check of the angular-momentum measurement on an articulated calibration body.

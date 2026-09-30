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

- Whether the importer anchored `world_ref` or introduced another external constraint.
- Whether Isaac's link velocity and COM/inertia tensor frame conventions exactly match this calculation; an independent calibration case has not been run.
- Source mass, COM, inertia and collision fidelity, including the 11 sphere fallbacks and the existing neck tensor flag.
- Upstream R2 file-level NASA-1.3 license confirmation and suitability of G1 as a fallback physics asset.

## Harness validation — 2026-09-30

This section supersedes the momentum *measurements* above; it does not alter `configs/iss_v0_momentum.json` or erase the original failed run. The same 10 s, 240 Hz, zero-g settings and predeclared limits were used throughout. A ratios are maximum deviation divided by initial total momentum (limits: P ≤ 0.001, L ≤ 0.01). B ratios are maximum total momentum divided by `Pref` and `Lref` (same limits). Structured results and full Isaac logs are under ignored `outputs/`.

| Body / derivative | Test A P ratio | Test A L ratio | A | Test B P ratio | Test B L ratio | B |
| --- | ---: | ---: | --- | ---: | ---: | --- |
| Calibration free box | 0 | 0.000819 | **PASS** | — | — | **NOT_RUN**: no internal joint |
| Calibration free two-link | 0.00000422 | 0.001562 | **PASS** | — | — | **NOT_RUN**: zero-drive calibration only |
| R2, `world_ref` kept | 0.0000670 | 0.002326 | **PASS** | 0.0000414 | 0.013671 | **FAIL**: angular |
| R2, `world_ref` removed | 0.0000670 | 0.002326 | **PASS** | 0.0000414 | 0.013671 | **FAIL**: angular |
| Unitree G1 29-DOF | 0.0000746 | 0.004488 | **PASS** | 0.00000547 | 0.005641 | **PASS** |

The calibration used a 2 kg free box (0.4 × 0.2 × 0.1 m), an authored COM offset of (0.05, 0.02, 0) m and a principal-axis rotation of 45° about X, plus two such boxes joined by a free, zero-drive Z revolute joint. Both received the Test A initial base twist. For the box, measured initial `P=(0.1000000015, 0, 0)` kg·m/s matches the 0.1 kg·m/s known answer; its initial `L=(0, -0.0002500, 0.0030833)` kg·m²/s. The two-link initial `P=(0.200000003, 0.03199999, 0)` includes transverse motion from the joint geometry. Both conserve P and L within the fixed limits over 2,400 steps. Their evidence is `outputs/iss_v0_calibration_result.json` and `outputs/iss_v0_calibration_final_result.log`.

Three harness faults caused misleading original results. First, a tensor-set articulation twist was cleared on the first explicit `SimulationManager.step`; a one-step warmup **before** setting initial conditions fixes this, as seen in the two-link first-step probe. Second, `RigidPrim.get_velocities()` already returns COM linear velocity, so adding `ω × COM offset` double-counted transport. Third, the tensor inertia already includes principal-axis orientation in the link body frame; rotating it by the COM quaternion a second time produced false angular drift. The shared `scripts/momentum_measure.py` now uses COM velocity directly and rotates tensor inertia by link world orientation only. The offset-COM and rotated-axis calibration isolated these frame bugs before the final robot reruns. Previous ratios above are historical, not validated gate measurements.

### Anchor diagnosis and independent control

The original URDF has inertial-free `r2/world_ref` joined to `r2/robot_world` by fixed `r2/fixed/world_ref/robot_world`. In the imported R2 USD, `/r2/Geometry/tn__r2world_ref_dH` is a parent Xform with no rigid-body, mass or inertia schema. Its child `/r2/Geometry/tn__r2world_ref_dH/tn__r2robot_world_eK` carries the articulation root, with mass 0.01 kg and diagonal inertia (0.0001, 0.0001, 0.0001) kg·m². The fixed `world_ref` joint is collapsed by import. No imported joint has an empty/world body target. With `--remove-world-ref`, the articulation root moves to `/r2/Geometry/tn__r2robot_world_eK`; both R2 momentum outcomes are numerically identical. Thus the suspected `world_ref` world anchor is not supported by the imported USD or the comparison.

The G1 control used the locally present official [`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros) checkout at commit `ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee`, file `robots/g1_description/g1_29dof.urdf`, SHA-256 `e1dc89366bf96aa33ba4ef211884f6d81320ddd2ac6f8b1ad82986e64008be31`. The checkout declares BSD-3-Clause. Its URDF has 40 links, 39 joints and 35.115142 kg authored mass; seven links have missing or identity-style inertias. Numeric URDF dimensions are interpreted as metres, masses as kg, and the imported USD is Z-up. It uses authored mesh and primitive collision geometry with self-collision disabled for this gate; source CAD accuracy, collision fidelity, COM/inertia measurements and actual contact behavior remain UNKNOWN. The pelvis articulation root has no world-side joint. Test B mapped each side's `shoulder_pitch`, `shoulder_roll`, `shoulder_yaw` and `elbow` joint to the same four-joints-per-arm command and config gains; all eight names matched. No new raw asset was downloaded or added to Git.

**Conclusion:** the original near-100% Test A loss was a harness initialization fault, with two additional tensor-frame calculation faults. After correction, both calibration bodies and G1 pass A; G1 also passes B. R2 passes A but exceeds B's angular limit by 0.003671, unchanged by `world_ref` removal. The remaining R2 B failure is specific to its imported model or its actuation/solver interaction under this test; its precise cause is UNKNOWN. R2 is not accepted for the ISS v0 momentum gate. Do not tune thresholds or call either R2 derivative a validated physics asset on this evidence.

Reproduce with `python scripts/make_r2_zero_g_derivative.py`, `python scripts/make_r2_zero_g_derivative.py --remove-world-ref`, then `C:\isaacsim\python.bat scripts/iss_v0_calibration.py` and `C:\isaacsim\python.bat scripts/iss_v0_momentum_test.py --urdf <URDF> --out <result JSON>` for each robot. G1 resolves from the local Unitree checkout above. The final robot JSON/log pairs are `iss_v0_momentum_kept_result.json` / `iss_v0_momentum_kept_final2.log`, `iss_v0_momentum_noworldref_result.json` / `iss_v0_momentum_noworldref_final2.log`, and `iss_v0_momentum_g1_result.json` / `iss_v0_momentum_g1_final2.log` under `outputs/`. Runtime wrapper exit codes and JSON verdicts were recorded separately; the R2 runs return a failing gate verdict.

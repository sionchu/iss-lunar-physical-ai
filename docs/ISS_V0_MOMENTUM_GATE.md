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

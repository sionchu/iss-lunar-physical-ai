# Robonaut 2 ISS zero-g derivative

Date: 2026-09-30. This is a scripted research derivative of the unmodified `assets/raw/nasa-urdf-robots/r2_description/robots/r2c6.urdf` from the NASA-1.3 lineage described in [R2_ASSET_AUDIT.md](R2_ASSET_AUDIT.md). The upstream file remains untouched and gitignored. The specific upstream file-level license confirmation remains UNKNOWN; preserve the NASA-1.3 license, source and modification notices if redistributing this derivative.

## Modification notice

`scripts/make_r2_zero_g_derivative.py` removes the fixed, leaf `r2/objects/argos_support/argos_support_link` and its joint to `r2/pelvis`. This removes a 1 kg ground offload support from the free-floating zero-g model. The script also replaces only equal-diagonal, zero-product placeholder inertias whose radius of gyration `sqrt(I/m)` exceeds 0.3 m. For each, it uses a uniform solid box with the existing mass and dimensions from the scaled collision mesh axis-aligned bounding box, or the visual mesh if collision is unavailable. If neither mesh parses, it uses a uniform solid sphere of radius 0.03 m. The inertial origin, mass, COM, joints, joint limits, and other inertias remain as authored. Mesh bounding boxes are geometric proxies, not validated CAD inertia; mesh geometry origin and orientation relative to the inertial frame are an uncertainty.

The script parses STL and COLLADA vertices with the Python standard library and records every old/new tensor, method and bounding box in `outputs/r2_derived/changes.json`. Its source SHA-256 is recorded there.

## Static audit

| Measure | Result |
| --- | ---: |
| Removed links / joints | 1 / 1 |
| Replaced inertias | 31 |
| Sphere fallbacks | 11 |
| Remaining placeholders with `k > 0.3 m` | 0 |
| Derived links / joints | 131 / 130 |
| Original / derived URDF mass | 215.4785694 / 214.4785694 kg |

The original audit's one missing inertial and `r2/neck_upper` triangle-inequality flag remain. This static result does not establish fidelity of source mass, COM, other inertias, collision meshes or zero-g dynamics.

## Regenerate and audit

From the repository root:

```powershell
python scripts/make_r2_zero_g_derivative.py
python scripts/audit_urdf.py outputs/r2_derived/r2c6_zero_g.urdf --json outputs/r2_derived/audit.json
```

Generated output stays under `outputs/` and outside normal Git history.

## Optional floating-root variant — 2026-09-30

Run `python scripts/make_r2_zero_g_derivative.py --remove-world-ref` to produce `outputs/r2_derived/r2c6_zero_g_noworldref.urdf` and `outputs/r2_derived/changes_noworldref.json`. This applies the same ARGOS removal and 31 inertia replacements, then removes the inertial-free `r2/world_ref` link and fixed `r2/fixed/world_ref/robot_world` joint. It verifies that the source structure matches the expected parent and child before editing. The existing no-option output and `changes.json` remain the default. The added variant has `r2/robot_world` as its URDF root; its authored mass and diagonal inertia are 0.01 kg and (0.0001, 0.0001, 0.0001) kg m². The change record includes the source SHA-256 and the four removed elements.

The Isaac importer collapses the original fixed `world_ref` joint. In the original imported USD, `/r2/Geometry/tn__r2world_ref_dH` is an inertial-free parent Xform; the articulation root is `/r2/Geometry/tn__r2world_ref_dH/tn__r2robot_world_eK`. In the optional variant, the root is `/r2/Geometry/tn__r2robot_world_eK`. Neither imported stage has a joint with an empty world-side body target. See [ISS_V0_MOMENTUM_GATE.md](ISS_V0_MOMENTUM_GATE.md) for the measured comparison; the root change alone does not establish physics suitability.

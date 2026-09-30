# Robonaut 2 asset audit — ISS v0 Slice A

Audit date: 2026-09-30. Candidate decision tested: ISS Robonaut 2 with climbing legs; Lunar Unitree G1 under the shared experiment contract. The supplied, unmodified `r2c6.urdf` is the primary full-body candidate. The reduced `r2c_sim_full_dynamics.urdf.urdf` removes the hands and articulated foot grippers and therefore is not an equivalent Slice A candidate.

## Provenance and license

- Derivative source: <https://github.com/gkjohnson/nasa-urdf-robots>, local commit `1f89ecba9112361cde5beda617575bdd7b811b24` (`git -C assets/raw/nasa-urdf-robots rev-parse HEAD`).
- Claimed upstream: NASA/JSC `r2_description` at `654f4f89ff8e802cb7f80c617f0d6dd04483f4b2`, linked by the derivative README: <https://gitlab.com/nasa-jsc-robotics/r2_description/tree/654f4f89ff8e802cb7f80c617f0d6dd04483f4b2>. The README says the files were converted from xacro using ROS Indigo. This is a lineage claim, not independent verification of the upstream archive.
- The clone contains **no bundled license text**. Its README says derivative URDFs carry the original license and links **NASA-1.3**. The [NASA Open Source Agreement 1.3 text](https://opensource.org/license/NASA-1.3) grants modification and redistribution under conditions, including keeping the same license, supplying the license and source, and identifying modifications. The declared “license forbids modification/redistribution” FAIL condition is not met on the available license evidence. Whether the upstream R2 release is still public, and whether its specific files carry the claimed license, remain UNKNOWN.
- Scale/frame and inertial provenance: URDF numeric values are treated as metres, kilograms and kg·m² by the parser; the original measurement/CAD derivation, coordinate-frame validation, mesh scale, collision suitability and contact fidelity are UNKNOWN. Visual and collision geometries are mostly meshes; importing them does not validate physics fidelity.

## Gate results

| Item | Status | Evidence |
| --- | --- | --- |
| Derivative source and pin | PASS | Local Git `rev-parse HEAD` returned `1f89ecba9112361cde5beda617575bdd7b811b24`; local README names NASA/JSC upstream pin. |
| License prohibits modified/derived redistribution | PASS | README identifies NASA-1.3; linked license §2A expressly grants modification and redistribution, subject to §3 obligations. Upstream file-level license confirmation is UNKNOWN. |
| More than half of links lack inertial data or have identity-style placeholders | **FAIL** | `python scripts/audit_urdf.py`: **99/132 (75.0%)**: 1 missing inertial and 98 equal-diagonal/zero-product-of-inertia candidates. This exceeds the predeclared threshold. |
| Tensor validity | FAIL | Same audit: one principal-moment triangle-inequality violation (`r2/neck_upper`); no non-positive-definite tensors or non-positive masses reported. |
| Climbing legs present | PASS | 7 revolute joints per leg, with limits recorded in the JSON joint inventory. |
| Leg end effectors present | PASS | 3 revolute gripper joints per leg (`joint0`, `jawLeft`, `jawRight`), each limited from -0.1 to 0.75 rad. |
| Arms and hands | PASS | 7 revolute arm joints and 18 finger/hand joints per side. Waist: 1; neck: 3. |
| URDF mass | PASS | Sum of authored link masses: **215.4785694 kg**. A published R2 mass comparison is UNKNOWN. |
| Isaac Sim 6.0.1 floating-base, zero-g import | PASS | `C:\isaacsim\python.bat scripts/isaac_import_r2.py` exited 0: import success, articulation found, 74 DOFs, PhysX link-mass sum **215.4785614 kg** over 131 links. Cache warnings occurred; see below. |

`scripts/audit_urdf.py` writes a per-link mass, COM, full tensor, principal moments, geometry types and flags to `outputs/r2_urdf_audit.json`, plus a human-readable table and all joint limits to stdout. Exact inertia repeats occur on 107 links in groups of three or more; repetition alone is flagged for review, while only missing/identity-style tensors count toward the predeclared majority gate. The 132-link model has 74 revolute and 57 fixed joints. Its visual geometry entries include 69 meshes, 2 cylinders and 1 box; collision entries include 69 meshes and 2 cylinders. The `r2/world_ref` link has no inertial element.

The reduced `r2c_sim_full_dynamics.urdf.urdf` has 65 links and 32 missing/identity-style inertias (49.2%). It has no hand/finger or foot-gripper joints, so it independently meets the missing-end-effector FAIL condition. The full `r2c6.urdf` is the fairer candidate for this decision.

## Commands and runtime evidence

```powershell
git -C assets/raw/nasa-urdf-robots rev-parse HEAD
python scripts/audit_urdf.py > outputs/r2_audit_stdout.txt
python -m py_compile scripts/audit_urdf.py scripts/isaac_import_r2.py
& C:\isaacsim\python.bat scripts/isaac_import_r2.py *> outputs/r2_isaac_import.log
```

The importer script sets `fix_base=False`, maps `package://r2_description`, writes converted output only under ignored `outputs/`, opens the stage, sets scene gravity magnitude to zero, looks for `ArticulationRootAPI`, and obtains DOF count and link masses from a live PhysX articulation tensor. The result is in `outputs/r2_isaac_import_result.json`; the full Kit stdout/stderr is in `outputs/r2_isaac_import.log`. Key output: `"import_success": true`, `"articulation_found": true`, `"dof_count": 74`, `"physx_total_mass_kg": 215.4785614013672`, `"physx_link_count": 131`, `"error": null`, process exit code `0`. The PhysX mass differs from the URDF sum by about 0.000008 kg, consistent with output precision; this is not a validation of physical mass accuracy.

The Kit log also records warnings such as `Local data store : failed to open index file c:/isaacsim/kit/cache/DerivedDataCache/cacheindex.dat`, plus `Failed to execute query: attempt to write a readonly database (code: 8)` and `Unable to initialize the OptiX cache database at path C:\Users\getch\AppData\Local/NVIDIA/OptixCache/optix7cache.db.` The import and PhysX query nevertheless completed. The script's `warnings` JSON array does not intercept Kit warnings; the redirected log is the warning record.

## Verdict

**FAIL — use G1 for ISS Slice A under the predeclared fallback.** The full R2 candidate exceeds the missing/placeholder inertia threshold (75.0%). The model's articulated climbing legs and grippers do not cure that physics-data failure. Do not promote this derivative asset to a canonical physics model on import success alone. Lunar G1 remains a separate embodiment using the same experiment contract; this audit does not validate G1.

UNKNOWN: current public availability and file-level license of original NASA/JSC R2 release (upstream URL above); source/validation of R2 mass, inertia, COM, scale, frames and mesh collision properties; published R2 mass comparison; G1 asset physics suitability.

## Gate revision — 2026-09-30 (supersedes the verdict above)

**Status: explicit post-result gate change, approved by the project owner.** The original verdict above is kept unedited as the record of the pre-declared gate.

### Why the pre-declared inertia gate was the wrong metric

The gate counted links ("more than half of links have missing/placeholder inertias"). Zero-g whole-body momentum is dominated by where mass and inertia sit, not by link count. Independent re-check of `r2c6.urdf` (same file, same placeholder definition: equal diagonal, zero products of inertia):

| Measure | Value |
| --- | --- |
| Placeholder links / all links | 98 / 132 |
| Mass carried by placeholder links | 2.907 kg of 215.479 kg (**1.3 %**) |
| Placeholder links heavier than 1 kg | **0** |
| Placeholder diagonal values | 0.001 (54), 0.0001 (39), 0.1 (4), 1.0 (1) |
| Placeholders with implausible radius of gyration (k = sqrt(I/m) > 0.3 m) | 32 |

All links above 1 kg (waist, backpack, pelvis, arms, shoulders, head, leg segments, feet) carry non-placeholder tensors. Their accuracy is still UNKNOWN; note that hip_pitch, knee and ankle_pitch share identical mass and inertia on both legs (copied values, source UNKNOWN).

### Revised gate

1. Mass-weighted: placeholder links must carry < 5 % of total mass. → **PASS (1.3 %)**
2. Handrail-critical links (leg grippers, feet, hands) must have physically plausible inertia. → **FAIL as authored**: each leg gripper jaw is 0.01 kg with I = 0.1 kg·m² (k ≈ 3.16 m), larger than an upper-arm link (≈ 0.044).
3. Non-robot links must be removed for zero-g use. → `r2/objects/argos_support/argos_support_link` (1 kg, I = 1.0) is a leaf fixed to `r2/pelvis`; by name it is the ARGOS ground gravity-offload support (identification is an inference, UNKNOWN until checked against upstream).

### Revised verdict

**CONDITIONAL PASS — R2 proceeds for ISS Slice A only through a scripted, documented derivative:**

- remove the ARGOS support link and its fixed joint;
- replace every placeholder whose k > 0.3 m (including the four gripper jaws) using one stated rule, with before/after values logged;
- keep the upstream file unmodified in `assets/raw/`; generate the derivative by script; record it as a NASA-1.3 modification.

Final acceptance still requires the ISS v0 zero-g momentum-conservation gate on the derivative. If that gate fails for asset reasons, the fallback remains G1.

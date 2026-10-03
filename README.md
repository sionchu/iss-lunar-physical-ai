# ISS → Lunar Physical AI

Research sandbox for space humanoid robotics and Physical AI using OpenUSD, NVIDIA Omniverse/Isaac Sim, and evidence-backed public space assets.

## Mission

Build one coherent simulation track rather than two unrelated demos:

1. **ISS Level** — microgravity humanoid dynamics, handrail locomotion, manipulation, and zero-g autonomy.
2. **Lunar Base Level** — reduced-gravity locomotion, mobile manipulation, terrain-aware autonomy, construction/maintenance, and base operations.

The ISS phase is the dynamics/control baseline. The lunar phase reuses the same telemetry, provenance, evaluation, and OpenUSD contracts. Reuse of the same humanoid hardware/model is a target, not an assumption: grasping suitability and lunar-walking suitability are separate gates, and task-specific controllers are allowed.

## Why this project exists

NVIDIA's Astra/Omniverse Labs ISS example shows that public NASA assets can be assembled into an OpenUSD ISS application with operational telemetry. NASA Astrobee provides an open zero-g robot/software reference. Isaac Lab has already been used in published zero-g Astrobee work, while OmniLRS provides an Isaac-Sim-based lunar simulator built from LRO terrain data. These pieces make an ISS-to-Moon research path technically credible without inventing a new simulator stack.

## Phase 0 — Evidence baseline

- Keep external evidence and saved links in Personal Knowledge Inbox (PKI).
- Keep project-scoped source selection and experiment definitions in this repository.
- Do not copy raw NASA/NVIDIA/third-party assets into Git before provenance and license review.
- Pin simulator versions, repository commits, asset hashes, experiment configuration, seeds, and success thresholds from the first accepted run onward.

## Phase 1 — ISS microgravity baseline

Authoritative v0 scope is defined only in docs/RESEARCH_BASELINE.md. The first accepted ISS slice is intentionally smaller than a full module: one floating humanoid, zero gravity, two collision-proxy handrails, and one deterministic transfer target.

After the v0 physics gate passes, expand toward free-floating attitude control, push–flight–park / handrail locomotion, reaction-aware grasping, and then tool/panel tasks. Exact acceptance criteria live in the research baseline.

## Phase 2 — Lunar Base

The lunar program targets the South Pole, but the first accepted lunar slice begins on rigid terrain at 1.62 m/s² to validate locomotion physics before integrating South-Pole DEM, procedural detail, lighting, or base infrastructure.

After the rigid-ground v0 passes, expand in order: lunar-gravity gait adaptation → South-Pole terrain/lighting → mobile manipulation → multi-robot logistics/construction. OmniLRS is the first environment reuse candidate; NASA DUST and Moon Base material are evidence sources for terrain, lighting, and operational constraints.

## System boundary

```text
Public evidence / NASA assets / papers
              ↓
        PKI evidence store
              ↓
     project source selection
              ↓
 OpenUSD scene + simulation contracts
              ↓
 Isaac Sim / Isaac Lab / Omniverse libs
              ↓
 controller / learning / autonomy
              ↓
 deterministic metrics + artifacts
```

Isaac Sim handles rigid-body, articulation, contact, sensor, and learning-oriented simulation. High-fidelity orbital mechanics, structural FEM, thermal/vacuum, radiation, and regolith-specific constitutive modeling remain separate modules or co-simulation candidates unless a validated implementation is added.

## Current state

This repository now contains the research baseline plus a reset-safe ISS zero-g momentum harness and asset diagnostics. Isaac Sim 6.0.1 is the current Windows runtime, but no humanoid model, ISS scene, lunar site, or task controller has passed all gates to become canonical yet.

See [docs/RESEARCH_BASELINE.md](docs/RESEARCH_BASELINE.md) for selected sources and acceptance gates. The canonical visual/scenario plan is [docs/ISS_TO_LUNAR_SHOWCASE_ARCHITECTURE.md](docs/ISS_TO_LUNAR_SHOWCASE_ARCHITECTURE.md).

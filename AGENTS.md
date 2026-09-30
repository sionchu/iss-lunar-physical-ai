# Project Agent Rules

- Apply Clean-v0, anti-bloat, evidence-first, and minimum coherent scope.
- Read this file, README.md, and docs/RESEARCH_BASELINE.md before changing project scope.
- Treat PKI as the external evidence/link store. This repo contains only project-scoped selections, contracts, code, configs, and reproducible experiment records.
- Never promote a community asset or third-party URDF/USD to canonical solely because it loads successfully.
- For every robot/environment asset, record source URL, license/provenance, version or commit, scale, frames, collision strategy, mass/COM/inertia source, and known uncertainty.
- Visual fidelity is not physics validity. Validate collision, scale, inertia, contact, gravity, and coordinate frames independently.
- Keep ISS and Lunar phases on one shared experiment schema; do not fork two unrelated architectures.
- ISS v0 scope is IVA microgravity. Do not add orbital mechanics until a task actually requires it.
- Lunar v0 scope is surface robotics at lunar gravity. Add regolith/deformable-terrain physics only when the experiment requires it and a validated model exists.
- Separate deterministic control/optimization baselines from learned policies. Compare both when practical.
- AI/RL results require measurable evidence: success rate, errors, forces/impulses, energy, collisions, and reproducible seeds/configs.
- Do not add FEM, thermal, radiation, or high-fidelity orbital models inside Isaac merely for completeness; use explicit co-simulation boundaries.
- Large downloaded assets and generated runs stay out of normal Git history unless intentionally managed with Git LFS and provenance metadata.
- Any meaningful cost, external submission/publication, destructive change, credential use, or broadened access is an approval gate.
- Before commit: inspect diff, verify links/config syntax, run available checks, and report UNKNOWN items rather than implying validation.

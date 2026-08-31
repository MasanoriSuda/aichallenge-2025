# Requirements: terminal Stop / normal-resume architecture audit

## Objective

Classify the first normal-authority loss at decision 2451 in
`output/20260831-175431/d1` without changing production authority.  Determine
whether the Stop and subsequent prolonged authority loss are caused by the
persistent lifecycle, direct candidate generation, the live single-SQP
approximation, or physical infeasibility.

## Frozen evidence

- baseline: `93de4923`
- decision: `2451`
- immutable interaction fingerprint: `883737710184574622`
- snapshot: `output/20260831-175431/d1/mpcc_architecture_snapshots/000000002451-0c43aac3e22b2e9e-cruise-side-neutral-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

## Required comparison

- A: recorded Cruise pipeline and seven-state SQP.
- B: independently rebuilt left/right current-world normal-avoidance candidates
  and the same seven-state SQP.
- C: bounded smooth/lattice normal-avoidance schedules and the same SQP.
- D: the same immutable C candidates with bounded offline multi-SQP.

All arms must retain the recorded walls, obstacle tube, vehicle footprint,
actuation limits, Stop suffix contract and world fingerprint.

## Prohibited changes

- No production authority, Mission resume rule, lease, grace period, timeout,
  fallback, solver tolerance, clearance or controller parameter change.
- Do not retain an old path merely because its Mission or candidate still
  exists.
- Do not classify all solver failures as physical infeasibility.

## Definition of done

- A--D are evaluated from the same frozen snapshot.
- The first causal failure is separated from downstream
  `steering-unreachable` and stuck-Recovery symptoms.
- A causal classification and the next structural action are recorded before
  production code is changed.
- Audit-only tests and the existing package test suite pass.

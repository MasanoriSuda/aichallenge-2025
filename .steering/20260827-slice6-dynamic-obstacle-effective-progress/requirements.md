# Requirements: Slice 6 dynamic-obstacle effective progress

## Objective

Remove the Cruise-to-Follow authority loss observed in
`output/20260827-204645/d1/autoware.log` by repairing the earliest coordinate
contract violated by the canonical dynamic-obstacle refinement.

## Failure-first evidence

- Decisions 1058--1071 retained the preceding Cruise artifact while Follow
  jobs were pending or rejected.
- Job sequence 481 rejected after the dynamic-obstacle refinement.  Its first
  wall-only state reported `theta=0.743 m`, target progress `2.195 m`, and
  longitudinal overlap `2.044 m`; the theta-only margin was therefore
  `-0.592 m`.
- The same canonical Follow formulation already defines physical along-track
  ego position as `theta + e_lag`.  The dynamic-obstacle row and its branch
  selector used `theta` alone.
- The false longitudinal overlap selected a 20-stage partial lateral escape,
  the QP reached maximum iterations, the old Cruise cursor exhausted at
  decision 1072, and Emergency braking became visible.

## Invariant

Every canonical longitudinal vehicle-separation constraint uses the same
physical progress coordinate, `theta + e_lag`.  A theta-only dynamic-obstacle
constraint must not be representable.

## Constraints

- Do not tune wall, gap, solver, steering-rate, velocity, or timing values.
- Do not add a fallback, retry, timeout, lease, grace, or normal authority.
- Keep lateral obstacle rows unchanged.
- Preserve the existing physical Follow hard-gap certificate.
- Keep generated result files uncommitted.

## Definition of Done

- A deterministic test fails on the theta-only formulation and passes only
  when branch selection uses effective progress.
- The QP row for longitudinal obstacle separation has unit coefficients on
  both progress and lag.
- The raw and effective wall-only progress values are independently visible
  in diagnostics.
- Focused tests, all package tests, and `make autoware-build` pass.
- A bounded `make dev2` no longer reproduces this coordinate-induced
  Cruise-to-Follow Emergency episode.

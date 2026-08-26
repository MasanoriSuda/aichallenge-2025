# Requirements

## Goal

Restore canonical Track/Cruise/Follow production authority across stationary
and predicted stop states without adding a fallback or relaxing a physical
command bound.

## Evidence boundary

- baseline HEAD: `992be2a`
- run: `output/20260826-095203`
- domains: d1 and d2
- first abnormal producer: every solved six-state Track/Cruise result is
  discarded as `execution artifact rejected: invalid-control-stage`
- visible consequence: `available=0/production_canonical=0` and the published
  command remains `speed=0`, `acceleration=-3`, `emergency=1`

## Repaired invariant

A row-certified internal virtual-progress input must be judged with the same
physical residual tolerance at solver output and immutable artifact validation.
A valid stop/hold singleton `[0, 0]` must not require an exact floating-point
zero from OSQP.  The same certificate must follow a predicted velocity state
through the exact physical-trajectory boundary; consumers must not silently
replace a certified lower-bound residual with an exact-zero requirement.

## Constraints

- Do not change solver settings, weights, speed, wall, vehicle, or clearance
  parameters.
- Do not add a fallback, lease, timeout, feature flag, or alternate authority.
- Acceleration and steering-rate physical command envelopes remain inset and
  fail closed.
- A virtual-progress result outside the certified physical tolerance remains
  rejected.
- Preserve ROS/evaluation contracts and the user-owned
  `aichallenge/result-summary.json` modification.

## Definition of done

- A deterministic test reproduces the zero-width hold plus solver-residual
  case and fails before the production fix.
- Artifact validation accepts a certified residual inside the solver's
  physical tolerance and rejects one outside it.
- Focused tests, complete package tests, source-contract tests and workspace
  build pass.
- A new moving run produces at least one physically certified and executed
  Track/Cruise plan in each active domain.

# Requirements: current-world Stop audit

## Frozen evidence

Source snapshot:

`output/20260831-134900/d1/mpcc_architecture_snapshots/000000001563-8903ab412b2e24f9-pass-side-negative-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

At decision 1563 the retained Pass artifact lost rate-resolved authority, its
terminal Stop collided with the wall, and no fresh normal authority joined.
The architecture comparison rejects A/B/C/D at the solver boundary.  Its Stop
arms are not evaluated because they incorrectly depend on a successful normal
solve, while production already evaluates an independent current-world Stop.

## Objective

- Remove the observation-only comparison's dependency on a successful normal
  solve when evaluating a current-world seven-state Stop.
- Keep the immutable snapshot, solver settings, physical proof and production
  authority unchanged.
- Re-run the frozen snapshot to distinguish physical infeasibility from an
  audit/candidate-generation blind spot.

## Non-goals

- No Mission resume rule, lease, grace period, timeout or fallback.
- No solver tolerance, wall clearance or vehicle-limit change.
- No production authority change in this Slice.

## Definition of Done

- A normal-solve failure cannot suppress the current-world Stop audit arms.
- Tests cover the failure mode.
- The frozen decision 1563 is reclassified with direct Stop evidence.
- Build and package tests pass.


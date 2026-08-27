# Requirements: Pre-entry semantic identity

## Objective

Remove the split-brain pre-entry problem in which a hypothetical ShiftOut or
Pass branch is assembled with the currently published Follow/Cruise semantic
intent.  The candidate's semantic intent must be fixed before any bounds,
wall profile, dynamic-obstacle contract or seven-state request is produced.

## Frozen evidence

- Source baseline: `c7708dc7`.
- Dynamic run: `output/20260828-003547`.
- Exact failing snapshot:
  `000000001132-shiftout-initial-solve-rejected/snapshot.yaml`.

The snapshot is labelled ShiftOut, but its source problem reports
`progress_wall_profile_diagnostic=rate-resolved-normal-scope-inactive`.  Its
first temporal state is structurally infeasible: at zero speed the affine
model keeps lateral position at `-1.018 m`, while state one requires
`[-1.605, -1.205] m` from a future spatial wall sample.

## Constraints

- Do not change a parameter, margin, solver tolerance, horizon, timeout,
  fallback, lease or authority rule.
- Do not widen a physical wall or opponent constraint.
- Resolve the prospective intent exactly once and use that value for both the
  source problem and the seven-state problem.
- Preserve current production authority; this is an observation-only branch
  admission repair.
- Add a unit contract for fresh direct Pass, same-side Pass continuation,
  paused same-side continuation and ordinary ShiftOut.

## Definition of done

- The prospective intent resolver has exhaustive unit coverage.
- `init_problem()` receives the same prospective intent later passed to the
  seven-state builder.
- Package build and complete package tests pass.
- A new `make dev2` run either admits an Overtake branch or emits a later,
  causally distinct frozen failure; it must not reproduce the same
  `rate-resolved-normal-scope-inactive` ShiftOut snapshot.
- Results and the architecture A--D classification are recorded before the
  next production change.

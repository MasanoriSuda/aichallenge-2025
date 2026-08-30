# Requirements

## Objective

Classify the remaining `invalid-actuation-sequence` failures emitted by the
new current-world Stop successor production edge before changing its behavior.

## Frozen evidence

- Baseline: `300e987f fix(mpcc): retain certified stop successor authority`
- Run: `output/20260830-183143/d1/autoware.log`
- Accepted reification crossed canonical normal authority repeatedly.
- Other accepted Stop proofs collapsed into the single reason
  `invalid-actuation-sequence`, including live Pass/ShiftOut failures and a
  later Recovery-override interval.

## Constraints

- Production authority and command values remain unchanged.
- No solver, tolerance, clearance, weight, timing, lease, grace, timeout,
  fallback or Mission rule change.
- Do not infer physical infeasibility from the aggregate reason.
- Preserve fail-closed behavior for every rejection.

## Exit criteria

- Every structural actuation-sequence rejection has a stable detailed reason,
  rejected index and relevant observed/bound values.
- Logs distinguish exact-trajectory shape, initial bounds, command index,
  within-interval command mutation, sample validity, duration, publication
  coverage and progress regression.
- Unit/source-contract tests and the full package test pass.
- A dynamic run identifies the first causal rejection family before any
  behavioral fix is proposed.

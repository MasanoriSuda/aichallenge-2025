# Requirements

## Objective

Identify why a newly solved five-state Overtake trajectory passes primal
normalization and extraction but is then reported as an incomplete exact
physical artifact on the first ShiftOut cycle.

## Constraints

- Do not add entry grace, retry, timeout, lease, or legacy authority.
- Do not tune OSQP, wall margin, weights, or physical limits.
- Do not weaken the exact artifact contract without field-level evidence.
- Preserve the user's generated `aichallenge/result-summary.json` change.

## Acceptance

- Every exact-artifact rejection names the failed invariant and stage.
- A failure-first test covers the observed contract boundary.
- Dynamic evidence distinguishes async-not-ready from a solved-artifact
  invariant mismatch.
- Only an evidence-backed structural repair may change behavior.

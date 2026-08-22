# Task list

## Failure-first and diagnosis

- [x] Reproduce mixed-unit false acceptance during physical-bound refinement.
- [x] Add row-wise residual/tolerance regression tests.
- [x] Map five-state stage 1..N lateral rows with accept/reject/malformed tests.

## Implementation

- [x] Preserve row-wise residual provenance in solver output.
- [x] Gate Track/Cruise shadow by the lateral metre-domain contract.
- [x] Remove observed-violation-based extraction tolerance.
- [x] Keep production authority and physical certificate unchanged.

## Rejected alternatives

- [x] Measure and remove same-cycle second-solve refinement.
- [x] Measure and remove reference-heading `e_y` preflight contraction.
- [x] Measure and remove linearized lateral-heading constraint rows.

## Validation

- [x] `make autoware-build` passes.
- [x] Full `multi_purpose_mpc_ros` test suite passes.
- [x] Four-wrap final shadow run completed.
- [x] `selected=1` remains zero.
- [x] `git diff --check` passes.

## Completion

- [x] Update validation and canonical specification.
- [x] Commit without `aichallenge/result-summary.json`.

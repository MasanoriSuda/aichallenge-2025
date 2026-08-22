# Slice 2b task list

## Failure-first proof

- [x] Add effective progress geometry contract tests.
- [x] Add typed MPCC actuation proposal tests.
- [x] Confirm focused tests fail before implementation (missing contract APIs).

## Implementation

- [x] Implement effective progress geometry resolver.
- [x] Store raw and effective geometry separately in `MpcProblem`.
- [x] Select geometry identity by formulation.
- [x] Feed effective cumulative distances to five-state physical certification.
- [x] Implement typed actuation proposal extraction.
- [x] Join shadow proposal with final command telemetry by decision ID.
- [x] Confirm `selected=0` and zero config/production authority changes.

## Validation

- [x] Focused tests pass.
- [x] Full package tests pass.
- [x] `make autoware-build` passes.
- [x] `git diff --check` passes.
- [x] Run repeated single-car simulation.
- [x] Confirm zero `solution heading unavailable` rejection.
- [x] Recompute certificate coverage and runtime budget.
- [x] Classify remaining physical rejections.

## Completion

- [x] Update validation/spec.
- [x] Commit without `aichallenge/result-summary.json`.

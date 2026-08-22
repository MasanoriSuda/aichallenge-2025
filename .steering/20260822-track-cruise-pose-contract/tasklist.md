# Task list

## Failure-first

- [x] Add exact five-state pose extraction tests.
- [x] Confirm focused test build fails before implementation.

## Implementation

- [x] Add typed five-state execution trajectory extraction.
- [x] Add optional exact-heading certificate input.
- [x] Connect exact heading to Track/Cruise shadow only.
- [x] Confirm production authority/configuration are unchanged.

## Validation

- [x] Focused and full package tests pass.
- [x] `make autoware-build` and `git diff --check` pass.
- [x] Repeat single-car simulation.
- [x] Compare physical reject category/rate with `output/20260822-135649`.
- [x] Confirm `selected=1` remains zero.

## Completion

- [x] Update validation and canonical specification.
- [x] Commit without `aichallenge/result-summary.json`.

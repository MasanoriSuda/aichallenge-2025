# Task list

## Failure-first

- [x] Add deterministic reason/format tests.
- [x] Confirm focused tests fail before implementation.

## Implementation

- [x] Add typed certificate diagnostics.
- [x] Populate diagnostics without changing validation decisions.
- [x] Emit structured evidence only on shadow outcome transition.
- [x] Confirm no config or authority change.

## Validation

- [x] Focused and full package tests pass.
- [x] `make autoware-build` and `git diff --check` pass after the final diagnostic-only fix.
- [x] Repeat single-car simulation.
- [x] Classify every shadow physical rejection category.
- [x] Confirm `selected=1` remains zero.

## Completion

- [x] Update validation and canonical specification.
- [ ] Commit without `aichallenge/result-summary.json`.

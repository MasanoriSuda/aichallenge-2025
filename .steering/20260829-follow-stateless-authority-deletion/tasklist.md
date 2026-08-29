# Task list

- [x] Freeze the repeated A-fails/B-succeeds architecture evidence.
- [x] Delete persistent Follow geometry evaluation.
- [x] Require exact current-world dynamic proof for Follow B admission.
- [x] Add deletion and certificate source-contract tests.
- [x] Run focused tests and full build (73 tests; 25 packages).
- [x] Run bounded `make dev2` and compare first-result latency/authority gaps.
- [x] Commit accepted implementation or remove rejected experiment.

## Classification

- Slice invariant: accepted. Follow begins with current-world B and exact
  dynamic proof is joined before Store admission.
- Overall race quality: not accepted. A later B candidate excludes its own
  current initial state at stage 0 and is correctly build-rejected.
- Next Slice: prevent the current-world candidate producer from emitting a
  stage-0 corridor that excludes the immutable current vehicle state.

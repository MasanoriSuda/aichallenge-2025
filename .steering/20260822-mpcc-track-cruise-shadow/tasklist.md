# Slice 2 task list

## Contract and pre-fix proof

- [x] Fix baseline commit and preserve unrelated working-tree state.
- [x] Record the earliest structural failure: Track/Cruise cannot build five-state metadata.
- [x] Add failing tests for shadow eligibility and warm-start compatibility.
- [x] Confirm the tests fail before production implementation.

## Implementation

- [x] Separate progress metadata availability from live formulation authority.
- [x] Add dedicated Track/Cruise shadow solver context.
- [x] Add same-formulation rolling warm-start/reset policy.
- [x] Add physical swept-wall certification for the shadow prediction.
- [x] Add command/prediction/timing/coverage telemetry.
- [x] Confirm shadow output cannot reach final authority or live solver state.

## Static validation

- [x] Focused foundation tests pass.
- [x] Full package tests pass.
- [x] `make autoware-build` passes.
- [x] `git diff --check` passes.
- [x] Diff audit finds zero production command/config changes.

## Dynamic validation

- [x] Run fixed single-car Track/Cruise trial (six laps).
- [ ] Run repeated/multivehicle trial if timing remains acceptable (deferred because single-car
  physical coverage already blocks promotion).
- [x] Measure metadata/solve/certificate coverage.
- [x] Measure mean/p95/p99/max callback time and consecutive overruns.
- [x] Verify no shadow result is selected or published.
- [x] Record decision and remaining Slice 3 blockers.

## Completion

- [x] Update steering validation evidence.
- [x] Update current implementation spec with the diagnostic-only behavior and promotion blockers.
- [x] Commit without `aichallenge/result-summary.json`.

## Promotion blockers found

- [ ] Unify circular seam stage geometry used by five-state dynamics and physical certification.
- [ ] Define an explicit MPCC actuation proposal instead of treating predicted velocity as the
  legacy target-speed command slot.
- [ ] Re-evaluate real hard-wall/swept-path rejections using the repaired common geometry.
- [ ] Bound or move shadow computation if repeated trials show callback overrun sequences.

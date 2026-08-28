# Task list

- [x] Freeze the controlled-comparison question.
- [x] Add a failing report-shape and missing-target-binding test.
- [x] Implement the audit-only A2 builder and report arm.
- [x] Run focused tests.
- [x] Re-run the production failure snapshot.
- [x] Record the resulting root-cause classification.
- [x] Run full build/package tests.
- [x] Commit the completed audit Slice.

## Validation

- Focused stateless binder test: passed.
- Architecture comparison GTest: 5/5 passed.
- Docker/colcon build: 25 packages passed.
- Package test: 52/52 test targets passed; 2047 tests, 0 failures.
- Comparison ran from `/tmp`; offline failure recording was kept separate from
  production architecture snapshots.

## Frozen-snapshot result

- `persistent-a`: wall-refinement solve rejected at row 170.
- `persistent-target-bound-a2`: the same row 170 failure with the same
  normalized violation. Restoring the target alone does not repair the
  retained geometry.
- `stateless-right-b`: SQP solved, but exact timed dynamic proof found a
  `-0.002373 m` overlap. This is a model/certificate mismatch for the simple
  all-side constraint.
- `physical-diagonal-right-f`: multiple candidates passed the SQP, exact wall,
  exact timed obstacle and terminal-successor proof.

Classification: the frozen failure is not physically infeasible.  Its
remaining cause is candidate construction: retained Mission geometry cannot
solve the wall-refined problem, while a simple stateless side row is not a
physical certificate.  A current-world physical diagonal candidate is a
complete witness.  Production authority remains unchanged in this Slice.

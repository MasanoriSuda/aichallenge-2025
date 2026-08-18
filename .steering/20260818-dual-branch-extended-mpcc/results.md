# Results

## Implemented

- The existing latest-only tactical worker now launches independent left and
  right production extended-MPCC solves concurrently.
- Each branch uses a deep `ReferencePath`, `BicycleModel`, gap-planner and MPC
  snapshot. Solver state and mutable Mission state are not shared by branches.
- Both results are joined into one worker result before selection or live
  publication.
- Selection rejects failed, non-finite and below-reserve solutions; applies
  current-side objective hysteresis; and retains the current side after the
  tactical no-return point.
- A failed dual solve is a soft miss: the legacy tactical result and current
  last-feasible execution path remain available.
- Startup and one-second worker logs expose enablement, per-side attempt/
  feasibility/objective/reserve and the atomic selection reason.

## Static verification

- `make autoware-build`
  - success, 25 packages
  - only existing setuptools deprecation warnings
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 28/28 CTest targets passed
  - 1298 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`
  - success
- `pre-commit run --files ...`
  - not run because `pre-commit` is not installed on the host

## Dynamic acceptance check

Run `make dev2` and confirm the one-second log named
`Overtake MPCC-lite async` contains:

- `dual=L1/...` and `R1/...` when complete Missions exist on both sides;
- a finite objective and nonnegative reserve for each solved side;
- `select=1/...` or `select=-1/...` only after a feasible branch result;
- current-side retention after no-return;
- no control-frequency drop while the worker evaluates both sides.

The dynamic race result remains the user's effect-confirmation step.

# Task list

- [x] Inspect current OSQP 0.6.2 API and per-cycle setup path.
- [x] Add persistent solver and MPC-layout warm-start shift component.
- [x] Connect the main MPC solve path and bounded diagnostics.
- [x] Add unit tests for workspace reuse, structural rebuild, warm-start shift
  and failure recovery.
- [x] Build `multi_purpose_mpc_ros` and run its test suite.
- [x] Record verification results and commit the scoped change.

## Definition of Done

- Unchanged QP structure performs one setup followed by numeric updates.
- Successful solves provide dimension-checked shifted primal/dual warm starts.
- Update/solve failure cannot leave a reusable partial workspace.
- Existing controller and package tests pass.
- No participant/evaluation interface changes.

## Verification

- `docker compose run -T --rm --no-deps autoware-build`: passed (25 packages).
- `ctest --output-on-failure` in the `multi_purpose_mpc_ros` build directory:
  26/26 passed.
- `ament_uncrustify` on the new header, source and test: passed.
- Dynamic `make dev2` race behavior is intentionally left to the requested
  user-side trial; enable the existing V2X debug logging to see the bounded
  `OSQP runtime` and `Control callback runtime` summaries.

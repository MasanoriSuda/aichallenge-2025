# Task list

- [x] Record the latest worker/callback timing evidence.
- [x] Add a tested compute-budget interval policy.
- [x] Add load-shedding parameters and startup/status diagnostics.
- [x] Enable the policy in development and cloud configurations.
- [x] Build and run focused tests.
- [x] Commit the scoped change.

## Definition of Done

- Normal MPCC-lite tactical evaluation starts at 5 Hz.
- Expensive results extend the interval, bounded at 0.30 s.
- The control callback never waits for the tactical worker.
- Existing physical guards and participant interfaces are unchanged.
- Unit tests, package build and formatting checks pass.

Dynamic acceptance remains a `make dev2` task.

## Verification

- `docker compose run -T --rm --no-deps autoware-build`
  - passed; 25 packages built.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - passed; 1192 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`
  - passed.

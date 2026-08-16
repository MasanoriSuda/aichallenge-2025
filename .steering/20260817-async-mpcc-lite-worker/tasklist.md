# Task list

- [x] Confirm the callback-overrun and tactical-timing evidence from the latest run.
- [x] Add a tested latest-only background worker primitive.
- [x] Add detached V2X planner and MPC tactical snapshot support.
- [x] Route MPCC-lite tactical refresh through the worker.
- [x] Add target/generation/phase/side/age admission checks and diagnostics.
- [x] Enable the async worker in the normal `make dev` / `make dev2` configuration.
- [x] Build and run package tests.
- [x] Record verification and commit the scoped change.

## Definition of Done

- The control callback never waits for an MPCC-lite worker result.
- At most one queued tactical snapshot exists.
- Old target/Mission results cannot be promoted.
- Missing/stale/failed worker output retains current safe execution.
- Existing hard guards and participant interfaces remain unchanged.
- Unit tests, package build and formatting checks pass.

## Verification

- `docker compose run -T --rm --no-deps autoware-build`
  - passed; 25 packages built.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - passed; 1161 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`
  - passed.

Dynamic acceptance remains a `make dev2` task. Confirm that
`Overtake MPCC-lite async` reports adopted results, bounded pending/replaced
counts and lower control-callback overrun frequency than the 18.7% baseline.

The first trial at `output/20260817-005244` logged `async=disabled` because the
enable flag was present only in `config_for_cloud.yaml`. The same flag is now
also present in `config.yaml`, which is selected by the normal MPC launch.

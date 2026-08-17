# Results

## Static verification

- `docker compose run -T --rm --no-deps autoware-build`
  - Success: 25 packages built.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - Success: 28 test targets, 1259 tests, 0 errors, 0 failures.
- `git diff --check`
  - Success.

## Dynamic baseline

The pre-change run is `output/20260818-003303`.

- P1 completed 77 laps.
- 65 clustered MPCC hard-wall authority releases remained.
- 108 of 184 hard-wall log samples were around `wp_id=313..33`, spanning the
  circular trajectory boundary.

## Next run

Run `make dev2` without changing the current tuning. Compare:

1. `MPCC solution hard wall contact` clusters, especially `wp_id=313..33`.
2. `Pass -> Recovery` versus `Pass -> Return -> Idle` counts.
3. Physical wall contacts and lap-time tail.

The expected result is fewer execution-time wall rejections. Physical wall
contact must not increase; this change does not weaken the footprint or margin.

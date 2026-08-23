# Validation

## Static gates

- `make autoware-build`: 25 packages completed successfully.
- Focused warm-start tests: 27/27 passed.
- Focused canonical async tests: 24/24 passed.
- `multi_purpose_mpc_ros` package: 1721 tests, 0 errors, 0 failures,
  0 skipped.
- `git diff --check`: passed.

The existing result scan still reports the unrelated stale
`build/joycon_contract_guard/package.xml` reference. It does not belong to this
Slice and the aggregated test result remains clean.

## Failure-first dynamic sequence

| Gate | Output | Earliest rejected invariant | Evidence |
|---|---|---|---|
| 1 | `output/20260824-040144` | clone-local intent re-derivation | 81 completed, all `intent-not-overtake-execution` |
| 2 | `output/20260824-041406` | clone-local full-context re-derivation | 122 completed, all sealed-context mismatch |
| 3 | `output/20260824-042519` | legacy warm-start intent set | 154 completed, all `invalid-current-context` |
| 4 | `output/20260824-043223` | producer passed; live proof coverage is next | canonical-ready and current-world-ready |

Each failure was repaired at its producer boundary. No configuration,
clearance, retry, fallback, timeout or lease was changed.

## Accepted dynamic gate

Domain 1 reached one typed Overtake episode:

```text
Idle -> ShiftOut -> Pass -> FollowPrepare -> Recovery -> Idle
```

The Recovery reason belonged to existing production behavior:
`physical target separation conflicts with wall bounds`. The new producer was
shadow-only and did not publish the command.

Observed async producer state before context invalidation:

- submitted: 158
- completed: 157
- worker exceptions: 0
- current identity rejects: 0
- submission rejects: 0
- snapshot failures: 0
- maximum observed worker compute time in the one-second status samples:
  30.484 ms
- callback overruns: 0

Aggregated shadow telemetry across 158 eligible cycles:

- sealed context / lateral / primal / actuation / trajectory / physical /
  canonical chain: 155
- current-world complete: 124
- stored: 123
- current-world rejection outcomes: course-frame unavailable 23, stage
  corridor violation 9, corridor horizon unavailable 1, initial pending 1
- exact actuation difference: 0
- live shadow main-thread work: at most 5.606 ms in the reported windows

## Authority decision

The producer Slice is accepted. Production promotion remains blocked because
current-world coverage is 124/158 rather than complete. Existing synchronous
Overtake solve/conversion/circuit/reentry/fallback remains until a separate
Slice closes that proof gap and can delete the old owner in the same change.

## Commands

```bash
make autoware-build

docker compose run -T --rm --no-deps autoware-command bash -lc \
  'cd /aichallenge/workspace && \
   ./build/multi_purpose_mpc_ros/test_race_mpcc_foundation && \
   ./build/multi_purpose_mpc_ros/test_follow_canonical_async'

docker compose run -T --rm --no-deps autoware-command bash -lc \
  'source /opt/ros/humble/setup.bash && cd /aichallenge/workspace && \
   colcon test --packages-select multi_purpose_mpc_ros && \
   colcon test-result --verbose'

make dev2
make down
```

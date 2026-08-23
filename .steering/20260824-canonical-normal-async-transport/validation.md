# Canonical normal async transport validation

## Static result

- Failure-first ShiftOut test failed before implementation with
  `IntentMismatch`.
- Focused transport suite passes 20/20 tests after implementation.
- Package suite passes all 40 CTest targets; repository test aggregation
  reports 1713 tests, 0 errors, 0 failures and 0 skipped.
- `make autoware-build` passes all 25 packages.
- `git diff --check` passes.

The existing `colcon test-result` scan also reports a stale unrelated
`build/joycon_contract_guard/package.xml` lookup error while still returning
1713 tests with zero errors/failures. No source or test failure is associated
with this Slice.

## Contract coverage

The focused tests prove:

- exact Track, Cruise, Follow, ShiftOut, Pass and Return identities are valid;
- Stop is rejected as a non-normal supervisor intent;
- a ShiftOut snapshot cannot be reinterpreted as Pass;
- result plan intent must match its immutable result identity;
- live consumption requires the same exact intent and intent generation;
- the old Follow namespace names the same canonical transport;
- existing mailbox sequence, context and typed-failure rules still hold.

## Commands

```bash
make autoware-build

docker compose run -T --rm --no-deps autoware-command bash -lc \
  'cd /aichallenge/workspace && \
   ./build/multi_purpose_mpc_ros/test_follow_canonical_async'

docker compose run -T --rm --no-deps autoware-command bash -lc \
  'source /opt/ros/humble/setup.bash && cd /aichallenge/workspace && \
   colcon test --packages-select multi_purpose_mpc_ros && \
   colcon test-result --verbose'
```

## Dynamic gate

Not required for this behavior-neutral transport Slice. The next Slice must
connect an Overtake shadow producer only, then collect dynamic coverage before
any production authority promotion. A successful build or mailbox test is not
authority evidence.

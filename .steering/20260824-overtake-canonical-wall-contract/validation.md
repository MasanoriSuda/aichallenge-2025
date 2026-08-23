# Validation

## Static

- Source deletion gate: 6/6 passed.
- Focused retained current-world suite: 11/11 passed.
- `make autoware-build`: 25 packages succeeded.
- `colcon test --packages-select multi_purpose_mpc_ros`: 1,736 tests,
  zero errors/failures/skips.  `colcon test-result` also printed the known,
  unrelated stale `build/joycon_contract_guard/package.xml` lookup.
- `git diff --check`: clean.

The new failure-first test proves that the same path is accepted with zero
additional clearance and rejected as `DelayPrefixBlocked` with `0.40 m`.
Negative clearance is rejected as `InvalidInput`.

## Dynamic Gate

Run: `output/20260824-072942` (`make dev2`, rebuilt install).

- Three canonical Overtake telemetry windows were observed.
- Total evaluated/eligible/context-complete: 76/76/74.
- Fresh physical wall proofs: 74 accepted, zero physical rejects.
- Complete current-world canonical selections: 71.
- Incomplete classifications: two pending/context cycles, one corridor horizon
  unavailable and two stage-corridor violations.
- Every telemetry window reported `wall_clearance=0.400m`.
- Production still emitted eight `legacy-normal-bypass` decisions and one
  exact-wall rejection moved ShiftOut to FollowPrepare.

## Gate decision

Accept the contract correction.  It is safe to begin a separate atomic
Overtake authority-promotion Slice: publish a complete canonical selection and
delete the converted/three-state normal owner for Overtake in the same Slice.
Do not weaken the wall contract or add an availability lease.

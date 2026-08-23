# Validation

## Static

- `make autoware-build`: 25 packages successful.
- `multi_purpose_mpc_ros`: 1723 tests, 0 errors, 0 failures, 0 skipped.
- New regression cases:
  - measured progress ahead of retained expected-current progress expands the
    required interval backward;
  - a broken retained progress chain fails closed.
- `git diff --check`: passed.

The existing stale `build/joycon_contract_guard/package.xml` result warning is
unchanged and unrelated.

## Dynamic

- Command: `make dev2`
- Output: `output/20260824-045351`
- Domain 1 reached `Idle -> ShiftOut -> FollowPrepare -> ShiftOut`.
- `course-frame-unavailable`: 0 log lines.
- Overtake canonical shadow: 396 eligible, 298 current-world complete, 283
  stored, 0 course-frame rejects.
- During the first stable ShiftOut interval, 121 consecutive cycles after two
  current-corridor rejects completed current-world proof.
- Worker exceptions, identity rejects, submit rejects, snapshot failures and
  callback overruns: 0 in the observed Overtake telemetry.

## Decision

Accept the course-frame repair. Do not promote Overtake production yet. The
next Slice must explain the transition-local progress/corridor failures before
authority promotion and legacy deletion.

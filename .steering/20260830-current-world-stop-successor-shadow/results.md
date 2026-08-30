# Results

## Static verification

- `make autoware-build`: passed, 25 packages.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed, 59/59.
- Stop physical adapter: passed, 22/22.
- retained current-world evaluator: passed, 55/55.
- single-authority source contract: passed, 88/88.

The direct tests cover exhausted-prefix acceptance plus fail-closed identity,
static-wall and dynamic-peer cases.  The source contract verifies that the
new evaluator cannot publish or mutate either candidate or executed Stores.

## Dynamic verification

Run: `output/20260830-180130`, `make dev2`.

D1 completed one full overtake episode:

```text
Idle -> ShiftOut -> Pass -> Return -> Idle
```

During episode 2, decision 4060 lost ordinary Pass authority because the old
terminal contingency was unavailable.  The production path emitted external
Emergency Stop, while the current-world shadow independently produced:

```text
intent=pass
normal=terminal-contingency-unavailable
result=accepted
physical=none/exact:accepted
control_wall=1/1
stop_wall=1/1
dynamic=1/1
states=166
authority=shadow
```

The run also exercised fail-closed outcomes including invalid identity,
invalid current world, static path blocked, control path blocked and invalid
lateral bounds.  Thus the evaluator does not turn every authority loss into
a synthetic success.

## Decision

Production promotion is justified for the exact same-intent, same-source,
current-world-certified successor case.  The next Slice must atomically
replace only the direct ordinary-authority-loss-to-external-Emergency edge.
It must not promote intent mismatches, wall/dynamic blocks, invalid physical
trajectories or missing current-world evidence.

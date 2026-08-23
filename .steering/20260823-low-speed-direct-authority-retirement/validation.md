# Validation

## Static and build gates

- Source deletion contract: 2/2 tests passed with external pytest plugins disabled.
- Package tests in the development container: 40/40 test programs passed;
  1,659 tests, zero errors, failures or skips.
- `make autoware-build`: 25 packages built successfully.
- `git diff --check`: passed.
- No parameter, solver tolerance, wall margin, timeout, lease, cooldown or feature-flag change.

## Deterministic replay

Input:

`output/20260823-214300-stop-authority-replay-v2/d1/rosbag2_autoware`

Current-controller output:

`output/20260823-low-speed-direct-replay/d1/autoware.log`

The replay excluded the recorded control command. The current controller was the only producer of
`/control/command/control_cmd`.

| Evidence | Before | After |
|---|---:|---:|
| `formulation=low-speed-direct` | 31 | 0 |
| `Low-speed pass shift control entered` | 1 | 0 |
| `prediction-unavailable` | 44 | 0 |
| Dynamic Escape five-state formulation traces | 33 | 62 |
| Dynamic Escape action traces | 5 | 45 |

At the first reproduced stopped-front decision, `d2` was again 6.30 m ahead. The new controller
published an `extended-mpcc-solved` five-state result with an 18-stage prediction and positive
acceleration. The wall handoff consumed that same prediction. It did not manufacture a prediction
and did not relax physical wall admission.

No callback overrun, actual contact, Reverse state or OvertakeLine Recovery transition was observed
in this bounded replay.

## Exposed pre-existing debt

Later in the replay the five-state OSQP solve reached its iteration limit. The existing production
chain then selected the three-state progress formulation 42 times and reported
`legacy-normal-bypass`. This is not caused by removing `LowSpeedDirect`; it was previously hidden
because the direct return bypassed both formulations and erased prediction evidence.

This Slice deliberately does not repair that second authority chain. Slice 5 must replace

```text
five-state unavailable -> three-state/legacy normal solve
```

with

```text
fresh certified five-state solution
-> current-world-certified retained five-state solution
-> explicit Emergency Stop
```

## Acceptance decision

Accepted. The obsolete stopped-vehicle normal authority is structurally unreachable and the
reproduced failure no longer destroys its own wall-certificate input. The newly visible
three-state fallback remains a separately named blocker for canonical Overtake/Dynamic Escape
promotion.

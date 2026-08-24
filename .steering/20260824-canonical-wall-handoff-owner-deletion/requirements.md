# Requirements

## Objective

Remove the downstream legacy wall-handoff owner that can reinterpret and
mutate an already current-world-certified canonical Overtake command.

## Evidence baseline

- Source baseline: `331846f`.
- Dynamic baseline: `output/20260824-114633/d1/autoware.log`.
- At decision 3586 the selected five-state ShiftOut plan reports a production
  wall contract of 0.40 m and retained proof reserve of 0.253 m.
- In the same callback, the old `dynamic-escape-exit` wall monitor evaluates a
  different outgoing prediction at 0.11 m and publishes
  `legacy-normal-bypass` with -3.0 m/s2.
- The resulting plant lag precedes DP source expiry and retained progress
  discontinuity.

## Invariants

1. A selected canonical ShiftOut/Pass/Return command is the only normal
   execution owner downstream of `MPC::get_control()`.
2. Solver, active-Overtake and DynamicEscape legacy wall-handoff gates cannot
   inspect, restore, or mutate that canonical command.
3. Obsolete DynamicEscape retained controls and exit-gate state are retired at
   the canonical Overtake ownership boundary.
4. Canonical physical wall/current-world proof remains mandatory before the
   command reaches this boundary.
5. Emergency supervisor and stuck Recovery remain able to override canonical
   normal control.
6. No wall margin, solver option, timeout, lease, grace, retry, or progress
   tolerance is changed.
7. User-owned generated results remain untouched.

## Definition of done

- Source and deterministic tests prove all legacy wall-handoff/escape paths
  are unreachable after canonical Overtake selection.
- The first accepted ShiftOut cycle publishes canonical fresh/retained output,
  not `legacy-normal-bypass` or `dynamic-escape-exit-wall-hold`.
- The downstream owner no longer causes the initial -3.0 m/s2 brake.
- Build/package tests pass.
- A bounded `make dev2` Gate records whether later progress discontinuity
  remains without the initial ownership conflict.

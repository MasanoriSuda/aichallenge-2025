# Requirements

## Objective

Move ShiftOut, Pass, and Return normal command authority from the coarse-stage
five-state MPCC execution plan to the existing rate-resolved six-state MPCC
pipeline.

## Dynamic evidence

`output/20260825-143421/d1/autoware.log` reproduced one Overtake entry:

- `Idle -> ShiftOut`: 1
- `ShiftOut -> Pass`: 0
- certified normal Overtake decision lines: 274
- Emergency Overtake decision lines: 359
- the episode ended as `ShiftOut -> Idle` after external Recovery
- Recovery incident completion took 53.77 s

The first failure chain is:

1. five-state retained plan publishes coarse-stage curvature-derived steering;
2. the next selected stage exceeds the 40 Hz steering reachability interval;
3. current-world revalidation rejects the plan as steering-unreachable;
4. repeated Emergency stops let the measured pose diverge from the immutable plan;
5. retained proof then rejects `initial-corridor-violation` and progress discontinuity;
6. Stuck Recovery takes ownership.

## Constraints

- Do not change controller weights, clearances, horizons, solver tolerances, or
  retry timing.
- Do not add a normal fallback, lease, grace period, clamp, or controller switch.
- The six-state plan must be solved from the command actually committed by the
  preceding control cycle.
- Missing current-world-qualified six-state evidence remains Emergency.
- Emergency and Stuck/gear/reverse Recovery remain separate supervisors.
- Preserve the ROS 2 and evaluation interfaces.
- Do not modify or commit `aichallenge/result-summary.json`.

## Exit criteria

- ShiftOut, Pass, and Return production traces identify
  `velocity-steering-progress-6state`.
- The Overtake branch cannot call the old five-state canonical producer.
- The next asynchronous Overtake problem is sealed only after the current
  output is committed.
- No normal command can fall through to the old Overtake five-state authority.
- Package tests and the repository build pass.
- A `dev2` run exercises Overtake without the reproduced steering-continuity
  Emergency cascade.

## Acceptance status

`output/20260825-160839/d1/autoware.log` proves the ShiftOut production join:

- 21 atomic ShiftOut admissions were physically certified;
- 17 ShiftOut cycles published a retained certified normal command;
- the execution contract reports
  `velocity-steering-progress-6state` and `canonical-rate-resolved-shiftout-retained`;
- rejected admissions identify physical wall contact or an invalid semantic
  steering sequence and fail closed; the original one-step steering
  continuity cascade is not present;
- the controller callback reported zero overruns in the captured shutdown
  window.

Pass and Return are implemented through the same intent-complete producer and
covered by deterministic tests, but this run did not reach either phase before
shutdown. Their dynamic evidence remains a follow-up acceptance item rather
than a reason to restore the retired five-state authority.

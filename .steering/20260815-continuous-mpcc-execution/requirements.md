# Continuous MPCC execution authority

## Background

The `20260815-121327` run entered nine overtake episodes, reached Pass six
times, but completed `Pass -> Return` only once. Soft planning misses repeatedly
changed the active OvertakeLine phase to FollowPrepare, replaced the selected
Frenet-DP path with a short measured-state prefix, and then allowed the locked
target's center distance to trigger SafetyBrake.

## Goal

Keep a recently wall/target/horizon-validated same-target/same-side Frenet-DP
path in control across ShiftOut, Pass, and a soft rolling-replan pause. The FSM
remains the supervisor for target identity, completion, hard faults, Return, and
Recovery.

## Constraints

- Do not change ROS topics, message types, services, launch entry points, or
  evaluation result schemas.
- Do not weaken physical wall contact, missing wall samples, target identity,
  current body overlap, position jump, forbidden waypoint, or solver Recovery
  guards.
- Do not reuse a DP path for a different target or pass side.
- A stale optimizer result may be reused only while a short runtime-validation
  lease is fresh and enough path remains.
- Preserve the existing contact-continuation lateral separation policy.

## Definition of Done

- Frenet-DP execution authority is phase-neutral across ShiftOut/Pass and an
  eligible DynamicMissionWait.
- DynamicMissionWait publishes the retained DP lateral sequence instead of an
  unconditional current-lateral straight prefix when it is still feasible.
- A validated DP path can suppress center-distance SafetyBrake only for the
  exact locked front target and only during the runtime-validation lease.
- Core unit tests cover active execution, rolling-replan retention, expiry, and
  hard-fault revocation.
- The package builds and tests in the repository Docker workflow.

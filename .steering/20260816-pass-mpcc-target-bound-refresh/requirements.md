# Requirements

## Goal

Reduce `Pass -> Recovery` caused by a rolling Frenet-DP path that is wall/kinematic-feasible
but no longer maintains the locked target's time-aligned physical lateral separation.

## Evidence

The `output/20260816-072344/d1/autoware.log` run reached `ShiftOut -> Pass` seven times,
but only one transition reached `Pass -> Return`; six transitions reached `Pass -> Recovery`.
Repeated failures include `optimized horizon escaped target separation bounds` and
Pass-continuation preflight loss after a rolling DP path had execution authority.

## Required behavior

- A pending same-target/same-side DP refresh must be checked against the locked target at each
  time-aligned control-horizon sample before atomic promotion.
- Samples outside the predicted longitudinal body-overlap window must not be constrained by the
  target.
- Samples inside that window must preserve at least physical center separation on the committed
  pass side.
- A rejected pending refresh must not mutate or revoke the previous last-feasible DP path.
- Recoverable side-contact handling remains owned by the existing contact-continuation path.
- Wall, lateral-acceleration, target continuity, emergency, solver and forbidden-waypoint hard
  guards remain unchanged.

## Scope

- `multi_purpose_mpc_ros` overtake pure core, controller integration and unit tests.
- No ROS interface, launch, result schema or evaluation-system changes.
- Solver-collapse recovery remains a separate deferred issue.

## Definition of Done

- Pure tests cover both a target-clear DP prefix and a target-crossing DP prefix.
- Atomic refresh promotion requires target-bound horizon feasibility.
- Existing package tests and build complete successfully.
- A dynamic run can report target-bound refresh rejection while retaining the active path; final
  Pass success-rate evaluation remains a user-side `make dev2` check.

# Requirements

## Objective

Determine why a Track/Cruise five-state MPCC command can remain freshly
certified while AWSIM produces a wall-contact penalty near waypoint 53.

## Scope

- Compare the failed six-lap run with successful Track/Cruise runs from the
  same source revision.
- Trace measured pose, trajectory-relative error, command, and abrupt speed
  loss around the first incident.
- Audit the physical wall certificate's assumptions and data provenance.
- Add observation or change production code only after a root cause is
  supported and has an explicit falsification condition.

## Constraints

- Do not tune wall margin, solver weights, steering gain, or speed.
- Do not add a fallback, feature flag, timeout, or special waypoint rule.
- Preserve the canonical five-state Track/Cruise authority and ROS contracts.
- Do not modify or commit `output/` or user-owned result artifacts.

## Dynamic evidence

- Failed run: `output/20260823-065700`
- Successful controls: `output/20260823-062054` and
  `output/20260823-064933`
- Rejected gain experiment: `output/20260823-072038`
- Unchanged-control six-lap acceptance: `output/20260823-075629`
- Observer-enabled one-lap false-positive check: `output/20260823-081219`

## Definition of Done

- Separate controller-commanded braking from an externally measured speed
  discontinuity without changing authority or commands.
- State explicitly what the static occupancy-grid certificate does and does
  not prove.
- Preserve a machine-readable incident path even when the expected AWSIM
  collision-condition topic is absent.
- Pass the focused monitor tests, package tests, and a dynamic no-false-positive
  run.

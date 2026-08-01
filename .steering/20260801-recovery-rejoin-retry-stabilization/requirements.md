# Requirements

## Purpose

- Prevent `LowSpeedRejoin` from continuing after it has captured the path laterally and then
  diverged across the path.
- Prevent simulation-only aggressive recovery from repeating indefinitely when the recovery
  candidate and surrounding safety snapshot are unchanged.
- Preserve one aggressive retry for a newly observed snapshot and re-arm retry when the pose,
  candidate, contact, gear, or V2X/static safety state changes materially.
- Remove the stale interpretation that an unlocked Forward fallback must remain the preferred
  direction forever.

## Scope

- `multi_purpose_mpc_ros` stuck-recovery pure core, ROS adapter, configuration, and unit tests.
- No changes to ROS topic names, message types, launch entry points, evaluation schemas, or
  overtaking behavior FSM.

## Constraints

- Simulation-only aggressive recovery remains bounded by all existing footprint, V2X, gear,
  speed, distance, and course-progress gates.
- An unchanged snapshot holds `SafeStop`; it does not disable monitoring or prevent a retry after
  meaningful external change.
- Existing user changes and generated `output/` artifacts are not modified.

## Definition of Done

- Rejoin lateral capture followed by configured regression transitions to bounded reassessment.
- One unchanged aggressive-retry snapshot cannot produce repeated retry cycles.
- Material snapshot change permits another retry.
- Forward fallback unlock no longer suppresses a valid Reverse candidate indefinitely.
- Package tests and `make autoware-build` pass.

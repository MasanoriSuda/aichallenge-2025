# Requirements

## Purpose

Remove the environment-specific `rear_safety.expected_v2x_vehicle_count`
setting.  The same submitted controller must work in `make dev2`, `make dev3`
and the competition environment without manually changing an expected peer
count.

## Required behavior

- Learn peer identities from structurally valid `/v2x/vehicle_positions`
  messages during the current race session.
- Treat a current message as complete only when it contains every learned ID.
- When tracked-message aggregation is enabled, require every learned ID to have
  a fresh, valid sample in the current Recovery epoch.
- Do not learn identities from messages with an invalid frame, timestamp,
  geometry, empty ID or duplicate ID.
- Reset learned identities only at the existing race-session tracking reset,
  not on each Recovery attempt.
- Continue to require the explicit `self_filter_mode` contract.

## Constraints

- Do not change ROS topics, message types, launch contracts or result schemas.
- Preserve the existing aggressive simulation override for incomplete V2X.
- Preserve the user's existing `aichallenge/result-summary.json` change.
- Old YAML files containing `expected_v2x_vehicle_count` may be accepted as
  legacy input, but the value must no longer control behavior.

## Definition of Done

- The parameter is absent from the shipped config and runtime config struct.
- Recovery completeness uses learned identity coverage rather than a numeric
  count.
- Focused tests cover complete, partial, unexpected and empty identity sets.
- Documentation no longer instructs users to edit a vehicle count per run.


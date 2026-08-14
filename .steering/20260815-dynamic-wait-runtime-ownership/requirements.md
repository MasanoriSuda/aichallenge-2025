# Requirements

## Purpose

Fix the unreachable `DynamicMissionWait` execution path observed in the latest
`make dev2` run.  A rolling tactical replan that has already entered
`DynamicMissionWait` must retain bounded forward authority while fresh Mission
candidates are assessed.

## Scope

- Make the runtime ownership condition explicit and unit-testable.
- Admit the wait executor only while overtake behavior is active and either:
  - tactical rolling replan is not active, or
  - `DynamicMissionWait` itself is active.
- Preserve the existing higher-priority atomic Mission replacement branches.
- Preserve all target, overlap, wall, and hard-fault guards in the executor.

## Out of scope

- Parameter tuning.
- Broadening forward authority during SafetyBrake or ordinary rolling replans.
- Changing ROS interfaces or evaluation contracts.


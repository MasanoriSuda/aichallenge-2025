# Design

## Root Cause

Large heading error during solver fallback latches a strict Reverse-only episode. In mixed wall
contact, every Reverse candidate can be rejected as `contact_worsened`. The existing Forward
unlock requires `wall_absent`, `current_footprint_clear`, and a clear V2X corridor, creating a
circular dependency: Forward cannot be evaluated until the vehicle is clear, but it cannot become
clear because no maneuver is selectable.

## Change

Add a pure `solver_reverse_deadlock_forward_probe_allowed` policy. It enables Forward candidate
evaluation only when all of the following are true:

- simulation environment and aggressive simulation recovery are enabled;
- Recovery is already active;
- a solver Reverse-only episode is latched;
- the wall classifier reports mixed contact;
- the course-progress guard is active;
- at least one aggressive retry has completed.

The policy does not unlock or command Forward directly. It only lets `evaluate_recovery_safety`
evaluate the existing short Forward primitives alongside Reverse. Existing logic then requires:

- improving or non-worsening swept-footprint contact transition;
- non-worsening course lateral error;
- complete boost and V2X observations;
- V2X clearance, including monotonically improving initial overlap;
- valid Drive gear before actuation.

## Diagnostics

Add `forward_probe=0/1` to existing Recovery transition and maneuver-selection logs. This is one
field in an existing log, not a new high-rate message.

## Scope

- `include/multi_purpose_mpc_ros/stuck_recovery_core.hpp`
- `src/stuck_recovery_core.cpp`
- `src/mpc_controller_cpp.cpp`
- `test/test_stuck_recovery_core.cpp`

No parameter, launch, topic, or message changes.

# Design

## Session-scoped identity discovery

`V2XGapPlanner` keeps a union of vehicle IDs seen in structurally valid V2X
arrays.  A valid newly observed ID expands this set.  The set survives
`begin_recovery_tracking_epoch()` so a partial array received after a collision
cannot redefine “complete” to a smaller count.  `reset_all_tracking()` clears
the set at the existing race-session boundary.

An empty learned set is not sufficient evidence for Reverse.  This preserves a
fail-closed safe mode when no authoritative peer observation has been received;
the existing simulation-only aggressive override remains unchanged.

## Completeness checks

- Current-message mode compares the latest valid message's ID set with all
  learned IDs.
- Tracked-set mode builds the set of fresh, valid IDs observed in the current
  Recovery epoch and compares it with all learned IDs.
- Identity coverage is implemented as a pure helper in
  `v2x_overtake_core`, allowing deterministic unit tests independent of ROS.

The comparison is identity-based rather than count-based, so `{P2, P3}` cannot
be mistaken for `{P2, P4}` merely because both contain two entries.

## Compatibility

No ROS interface changes are made.  YAML-cpp ignores the removed legacy key,
so older configs continue to load, but `expected_v2x_vehicle_count` no longer
affects execution.  `self_filter_mode=excluded` or `vehicle_id` remains
mandatory when Reverse actuation is enabled.


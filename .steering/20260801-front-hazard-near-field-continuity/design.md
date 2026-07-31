# Design

## Approach

Add a pure front-hazard continuity resolver in `v2x_overtake_core` and call it for the currently
held V2X target during behavior classification.

The resolver produces two independent facts:

1. `near_field_conflict`: the same valid target is within the configured rear-clear radius and
   overlaps the inflated lateral danger band in either the local or course-relative frame.
2. `rear_clear`: the target is at least the configured distance behind in the course frame when
   available, otherwise in the local frame, and is not a near-field conflict.

While the existing hold is active, `near_field_conflict` refreshes its existing 0.25 s deadline.
This makes the hold observation-driven rather than a longer fixed timer. Once the target moves
laterally clear, becomes safely moving ahead, or is genuinely rear-clear, current release behavior
is retained.

## Scope

- `include/multi_purpose_mpc_ros/v2x_overtake_core.hpp`
- `src/v2x_overtake_core.cpp`
- `src/mpc_controller_cpp.cpp`
- `test/test_v2x_overtake_core.cpp`

No parameter or interface changes are required. The existing
`v2x_front_hazard_rear_clear_distance` is also the bounded near-field distance.

## Expected Effect

The front/side/rear classification seam no longer creates a one-cycle acceleration window against
a nearby stopped vehicle. Normal side-by-side passing remains possible after physical lateral
clearance because the hold is no longer refreshed outside the inflated danger band.

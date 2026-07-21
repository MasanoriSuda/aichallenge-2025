# Design

## Scope

The change is confined to the participant MPC/V2X controller. ROS topics, message types, Domain
separation, AWSIM services, and evaluation result contracts are unchanged.

## State flow

1. Latch the stationary front/side target observed during start-grid grace.
2. Defer the emergency-follow return only while evaluating that target for breakout.
3. Resolve the breakout side from the existing staggered lateral positions. A visible offset
   beyond `v2x_start_grid_breakout_side_deadband` fixes the pass to that side. If both vehicles
   are nearly aligned inside the deadband, evaluate both sides instead of rejecting the breakout.
   Once selected, latch that side for the target; do not recompute it from changing relative
   lateral positions during ShiftOut.
4. Run the existing gap planner with inflated vehicle geometry and wall bounds on the eligible
   side or sides.
   Start-grid evaluation ignores the generic candidate-target lateral-acceleration estimate,
   because that estimate points at the corridor center while the executing OvertakeLine only moves
   from the existing staggered position to its bounded lateral goal.
5. If the side corridor passes the remaining checks, enter `Overtake`, lock that target, and let
   the overtake line own lateral motion.
6. While the remaining lateral shift completes, retain at least the breakout entry speed instead
   of decelerating to Follow. Release the full race reference after reaching the pass-side line.
7. If no executable corridor exists, return immediately to `SafetyBrake`.

## Isolation

- Controlled by `v2x_start_grid_breakout_enabled`.
- `v2x_start_grid_breakout_side_deadband` is independent of the OvertakeLine target-separation
  requirement; the former only resolves an initial side and does not prove pass completion.
- Requires active grace, a front+side classification, the latched initial target, and stationary
  target speed.
- The normal `v2x_overtake_guard_min_front_distance: 5.0` is not changed.
- Overtake-forbidden waypoint zones remain authoritative.

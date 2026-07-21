# Design

## Scope

The change is confined to the participant MPC/V2X controller. ROS topics, message types, Domain
separation, AWSIM services, and evaluation result contracts are unchanged.

## State flow

1. Latch the stationary front target observed during start-grid grace. Side classification is not
   an entry prerequisite because it can arrive one V2X update later; the gap planner still checks
   every received vehicle before execution.
2. Defer the emergency-follow return only while evaluating that target for breakout.
3. Before side lock, evaluate both collision-inflated side corridors. Prefer the feasible side
   with the larger planner corridor width; use the nearest-front geometric preference only for a
   width tie. The initial stagger is not proof that the same side is open because the third kart
   and wall may occupy it. Once selected, latch that side for the target; do not recompute it from
   changing relative lateral positions during ShiftOut.
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
8. Treat the grace duration as an entry window. If the same target already owns ShiftOut/Pass when
   the window expires, preserve its target and selected side until rear-clear or line recovery.
9. While the behavior remains a validated start-grid Overtake with an executable gap, do not let
   the close-front risk metric alone reset OvertakeLine. An explicit SafetyBrake, lost gap, blocked
   zone, solver failure, or other execution guard still resets it.
10. Let the dedicated breakout behavior remain the sole longitudinal reference owner. OvertakeLine
    still owns the lateral target but must not add the generic locked-target front cap on top of
    the breakout reference.

## Isolation

- Controlled by `v2x_start_grid_breakout_enabled`.
- `v2x_start_grid_breakout_side_deadband` is independent of the OvertakeLine target-separation
  requirement; the former only resolves an initial side and does not prove pass completion.
- Requires active grace, a front classification, the latched initial target, and stationary target
  speed for a new entry. Post-grace continuation additionally requires the same active line target.
- The normal `v2x_overtake_guard_min_front_distance: 5.0` is not changed.
- Overtake-forbidden waypoint zones remain authoritative.

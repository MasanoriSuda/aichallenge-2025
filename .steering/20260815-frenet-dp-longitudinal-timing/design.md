# Design

## Scope

This is the next MPCC-lite increment, not a full vehicle-dynamics MPCC replacement.  The
existing lateral Frenet DP remains the path solver.  Its target corridor is solved several
times with different ego arrival speeds so longitudinal timing and lateral feasibility are
chosen together.

## Planning flow

For each assessed pass side:

1. Construct closing-speed profiles: maximum attack, midpoint, configured minimum, and
   zero-closing hold. Duplicate speed profiles are removed.
2. Ask `V2XGapPlanner` to time-align the target footprint with each profile's bounded ego
   speed instead of the reference waypoint speed.
3. Build the same target-bound plus robust-wall DP corridor used by the current planner.
4. Solve the same-side Frenet DP for each profile.
5. Find the minimum lateral DP cost, admit profiles within a configured cost slack, and
   choose the fastest admitted profile. If the attack profile is substantially worse, the
   lower-cost timing wins.
6. Pass the selected closing speed into the existing kinematic rollout and Mission
   candidate. Prediction and execution therefore use the same longitudinal assumption.

If no timing produces a feasible DP corridor, planning retains the legacy nominal gap and
existing hard rejection path. No stale or fabricated path is installed.

## Authority boundary

The timing selector changes only candidate construction. Existing Mission freezing,
same-target/same-side atomic refresh, DP execution authority, and the 0.20 s runtime lease
remain unchanged. Emergency, physical wall/body, solver and forbidden-waypoint gates retain
priority over the selected timing.

## Parameters

- `v2x_overtake_mpcc_frenet_dp_longitudinal_timing_enabled`
- `v2x_overtake_mpcc_frenet_dp_longitudinal_cost_slack`

The cost slack is in normalized DP cost units. It prevents a slightly faster profile from
winning when it requires a materially worse lateral path.

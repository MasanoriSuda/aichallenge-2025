# MPC V2X Local Path Planner Requirements

## Purpose

Gate2 and race-like stopped/slow vehicle avoidance should not rely on MPC obstacle constraints alone. The controller must create an explicit local lateral target path before asking MPC to track it.

## Scope

- Keep the existing MPC solver, vehicle model, control output topics, and baseline CSV trajectory tracking.
- Add a low-speed V2X local path planner for stopped/slow vehicles.
- Use V2X vehicle positions and reference path coordinates to choose a pass side.
- Override MPC lateral reference targets during low-speed avoidance.
- Fall back to follow/safety brake when no feasible side pass path exists.

## Constraints

- Do not change ROS interface contracts such as `/control/command/control_cmd` or `/v2x/vehicle_positions`.
- Keep the old V2X gap planner available for existing behavior while the local planner is validated.
- Make the new behavior config-gated.
- Avoid gate2-only coordinates or hard-coded waypoint ranges.

## Done Criteria

- `use_v2x_local_path_planner` can enable the new local path logic.
- Low-speed avoidance enters only when a side pass target path is feasible.
- MPC `xr[e_y]` is driven by the local side-pass target across the horizon.
- `make autoware-build` completes.

# MPC V2X Local Path Planner Design

## Current Problem

The current V2X gap planner changes lateral constraints only where an obstacle overlaps the MPC horizon. Before that overlap, MPC keeps tracking the fixed CSV trajectory. If that trajectory is on the blocked side, the vehicle initially moves the wrong way and then reacts too late.

## New Structure

```
V2X positions
  -> stopped/slow vehicle local path planner
  -> explicit lateral target sequence
  -> MPC lateral reference xr[e_y]
```

## Planner Behavior

1. Find the nearest stopped/slow V2X vehicle ahead in the reference path frame.
2. Inflate the vehicle laterally by V2X vehicle radius and prediction margin.
3. Compute left and right free intervals between wall bounds and the inflated vehicle.
4. Select the configured pass side, or the wider feasible side in auto mode.
5. Generate a smooth lateral target:
   - current lateral error to pass-side gap center before the vehicle,
   - hold gap center while passing,
   - return to base reference after clearance.
6. Mark every MPC horizon point as target-active so the vehicle starts moving toward the chosen side immediately.

## Integration

- `evaluate_v2x_behavior()` uses the local planner for low-speed avoidance feasibility when enabled.
- `init_problem()` uses local planner output to override `xr[e_y]`.
- Existing gap constraints remain available for other modes.

## Risk

The first version still uses a lateral target sequence rather than publishing a full ROS trajectory. It is intentionally minimal to reduce blast radius and validate the planning structure first.

# Requirements

## Purpose

The dual extended-MPCC worker is enabled and healthy, but the first dynamic
run published `dual=L0/R0` for every sample. Both side evaluators required a
complete `selected_mission`, while the executable tactical results in this
course were usually bounded receding/progressive prefixes.

## Requirements

- Prefer a feasible complete selected Mission for each side.
- When no complete Mission exists, evaluate the side's feasible receding
  prefix with the same production extended MPCC.
- Fall back to a feasible selected progressive prefix when the separately
  published receding prefix is unavailable.
- Preserve `progressive_entry=true`; a prefix must not claim rear-clear or
  Return completion.
- Reuse the existing progressive entry/replacement admission at live commit.
- Do not use `entry_setup_mission`, because it prepares longitudinal speed and
  explicitly does not own a lateral line.
- Publish the branch source and rejection reason in the compact worker log.
- If neither side has a candidate or both solves fail, retain the existing
  tactical decision and last-feasible execution path.
- Do not change ROS interfaces or Recovery behavior.

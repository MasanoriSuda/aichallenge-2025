# Design

## Failure mechanism

`resolve_physically_validated_mpcc_execution_trajectory()` proves current wall-bound and static-footprint feasibility. It does not prove that the resampled trajectory is continuously reachable from the measured `e_y`, `e_psi`, and lateral velocity.

The controller previously set `solved_execution_wall_authority_active=true` immediately after that wall check. When normal DP authority was absent, the unpromoted trajectory became `solved_execution_bridge_active` and was passed to the execution horizon even if no stitch/reachability handoff had run.

## Change

Add a pure `resolve_solved_execution_bridge_authority()` policy. A solved trajectory may bridge only when all of the following hold:

- the controller is in an active execution phase;
- normal DP execution authority is not already active;
- the source passed physical validation;
- a handoff was requested;
- atomic promotion succeeded;
- the promoted trajectory is available;
- no runtime hard fault exists.

Physical validation remains a prerequisite but is no longer sufficient. Existing DP authority continues independently. A nominal wall warning is cleared by the solved source only when that connected promoted source actually owns the bridge.

## Degradation

If admission fails, the solved source remains pending and the existing Mission/DP/fallback path handles the cycle. The controller does not execute the disconnected source and does not convert it into a stop command directly.

## Compatibility

The change is internal to `multi_purpose_mpc_ros`; ROS and evaluation interfaces are unchanged.

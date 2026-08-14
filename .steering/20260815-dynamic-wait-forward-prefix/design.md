# Design

## Current-side forward prefix

`FollowPrepare/DynamicMissionWait` remains a tactical replan state, but it no
longer means "no lateral authority".  Each control cycle constructs a short
constant-current-lateral prefix and validates it against the current track
bounds, static-wall footprint and lateral-acceleration limit.  If the robust
wall margin cannot be maintained, the existing physical wall margin is tried;
failure leaves the prefix inactive and existing hard-fault handling wins.

This is intentionally not a replay of the invalidated Mission generation.  It
holds the current physical side while left/right candidates are regenerated.

## Longitudinal authority

The forward-prefix speed policy has three outcomes:

1. Hard fault or current body overlap: reject; existing Recovery handling owns
   the response.
2. Current body is separate but the predicted sweep is not: retain the prefix
   laterally, but cap closing at the configured unlatched value and add no
   speed floor.
3. Current and predicted physical footprints are separate: keep the frozen
   Mission closing request, bounded by the configured ShiftOut maximum and
   vehicle speed, and expose it as a reference floor subject to all hard MPC
   limits.

## Replacement priority

The existing `DynamicMissionWait` resolver already evaluates alternate before
current replacement and hold.  That priority is retained.  The new prefix is
used only for the `Hold` outcome, so a fresh fully preflighted alternate is
committed atomically on the same cycle it becomes admissible.

## Timing-profile diagnostics

`timing_profiles=0/4` is treated as evidence that no dynamic corridor was
formed, not as permission to fabricate a feasible trajectory.  The forward
prefix supplies a finite execution fallback while the real planner continues
sampling; hard corridor feasibility is not weakened.

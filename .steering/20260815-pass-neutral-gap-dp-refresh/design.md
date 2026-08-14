# Design

## 1. Target-bound lifecycle

Add an explicit `hard_failure` input to the lifecycle resolver.

- `target_bound_failure`: start/resume the original cumulative hold and clear
  the fresh-solution stability timer.
- `fresh_horizon_active`: retain the hold until the fresh horizon remains
  stable for the configured release time.
- neither: retain lifecycle bookkeeping as a neutral planner/tactical gap, but
  set prefix execution false.
- `hard_failure`: revoke the hold immediately.

The original start timestamp and traveled-distance origin remain unchanged
through neutral gaps, so a later target conflict consumes the original budget.

## 2. Rolling DP refresh source

The behavior planner already produces `mpcc_receding_mission` from the current
vehicle state.  It can be feasible when the full rear-clear/Return Mission is
not.  Publish this same-side prefix in a dedicated behavior-output field.

At execution time, prefer the fresh receding prefix for DP-only refresh.  Fall
back to the complete same-side Mission candidate when no prefix is available.
Do not use the prefix for atomic Mission replacement; only replace the active
distance-domain lateral reference.

The existing refresh resolver continues to enforce:

- active ShiftOut/Pass execution;
- exact locked-target identity;
- unchanged pass side;
- fresh prediction lease;
- minimum refresh interval;
- newer source timestamp;
- valid monotonic DP path.

## Expected dynamic evidence

- Neutral cycles no longer log `revoked by non-target horizon failure`.
- Explicit hard failures log a distinct hard revocation.
- `OvertakeLine DP execution rolling refresh` occurs repeatedly during a
  continuously observed Pass rather than only at initial admission.
- DP age remains close to the replan interval instead of growing to 4--5 s.
- `Pass -> Return` becomes possible without weakening wall guards.

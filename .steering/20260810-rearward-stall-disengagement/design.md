# Design

## Approach

Add a bounded `rearward progress-loss disengagement` policy to
SafeSeparation.

The policy is admitted only after `SideBySideCommitted` when:

- the locked target remains slightly behind ego but has regained a configured
  distance from the best observed relative position;
- no forward progress has been observed for a configured duration;
- the current vehicle footprints remain separated;
- the current execution corridor is not blocked; and
- no physical/runtime hard fault is active.

Once admitted, the lateral Mission and side stay frozen.  The longitudinal
reference becomes `target speed - disengagement delta`, allowing the target to
move safely ahead without crossing laterally through it.  When the target is
continuously beyond the existing front-clear distance, the policy requests the
existing dynamic Mission revalidation path.  Normal rear-clear still returns
immediately.

The disengagement has its own short timeout.  It does not turn prediction loss,
wall contact, current-body overlap, emergency braking or solver recovery into
soft conditions.

## Why not early Return

The normal return-corridor guard excludes the locked target by design.  At
`target_s=-0.6 m`, a direct Return could therefore cross the target even when
the non-target return corridor is reported clear.  This change keeps the same
side until longitudinal separation is restored.

## Configuration

- `v2x_overtake_safe_separation_progress_loss_disengage_enabled`
- `v2x_overtake_safe_separation_progress_loss_stale_sec`
- `v2x_overtake_safe_separation_progress_loss_regression_distance`
- `v2x_overtake_safe_separation_disengage_speed_delta`
- `v2x_overtake_safe_separation_disengage_max_sec`

Initial competition-simulation values are intentionally bounded and target
the measured failure rather than globally weakening the pass guards.

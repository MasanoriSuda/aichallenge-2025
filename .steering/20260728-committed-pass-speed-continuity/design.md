# Design

## Front-cap ownership

`can_release_overtake_front_cap()` originally required an unconstrained execution horizon for
the initial release. This steering introduced an existing-release hold through a lateral horizon
clamp when all of the following remain true:

- phase is `Pass`;
- front-cap release was already active;
- the physical front-overlap exclusion was latched;
- current lateral separation remains above the reapply threshold;
- the generated path is physically feasible; and
- the actual footprint is not in wall contact.

This separates a lateral reference clamp from locked-target longitudinal speed ownership without
removing wall or collision guards.

The follow-up steering `20260728-pass-initial-cap-release` extends the same policy to initial
release, but only at the full physical clearance threshold.

## Committed-pass reference floor

When the same committed/laterally-clear `Pass` sees its locked target at or below the configured
slow-target threshold, `OvertakeLine` publishes a configurable minimum velocity reference.
The MPC applies it as:

`reference = min(existing_hard_upper_bound, max(reference, committed_pass_floor))`

It is not a lower input bound. Existing front-risk, EmergencyBrake, curvature, acceleration,
wall, return-corridor, and recovery limits therefore retain priority.

## Configuration

- `v2x_overtake_committed_pass_speed_floor_enabled`
- `v2x_overtake_committed_pass_min_speed`

The existing `v2x_moving_front_speed_threshold` is reused as the slow-target threshold so the
new policy agrees with existing moving/stopped classification.

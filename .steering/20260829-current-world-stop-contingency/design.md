# Design

## Physical adapter

Add a deterministic exact Stop rollout which starts at the current
latency-compensated seven-state pose. It executes the command which is about to
be published for exactly one `publication_interval_sec`, then brakes to zero.
Steering rate is zero during the braking suffix, matching a hold-last-steering
Emergency transition. Course curvature and wall bounds are sampled from the
immutable execution artifact at the evolving progress state.

## Current-world proof

When the normal retained suffix is only current-stage clear, validate the Stop
trajectory against:

- exact nonlinear trajectory invariants;
- static swept-footprint wall clearance;
- current dynamic-obstacle tubes;
- current Follow hard-gap evidence when applicable.

Only a complete proof may set `terminal_stop_certified=true`.

## Authority

The canonical contract permits a partial retained horizon only when that flag
is present. Fresh candidates still require a complete horizon. The production
adapter independently verifies the same relation.

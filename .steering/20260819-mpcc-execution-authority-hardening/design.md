# Design

## Observed failure

The 2026-08-19 run retained a DP path with `refresh=0` and `age=2.05 s` although
`v2x_overtake_mpcc_frenet_dp_last_path_max_age_sec` was `0.50 s`.  During that
retention the frozen Mission goal was `0.74 m`, while the first DP target reached
`3.05 m`.  Extended MPCC had already entered its failure circuit and the controller
continued with the same reference through the 3-state fallback.  The vehicle then
reached `e_y=3.066 m`, `e_psi=0.514 rad`, and a measured wall distance of `0.001 m`.

## Safety contract

### Source freshness

`maximum_path_age_sec` is an absolute execution age. Runtime validation may bridge
target-prediction jitter while the source is fresh, but cannot replace an optimizer
refresh indefinitely.

### Reference trust envelope

Before a DP reference is used, constrain every stage to the intersection of:

- the current-side Mission envelope around the frozen corridor goal;
- the configured maximum same-side lateral adjustment around each nominal Mission
  stage;
- the already computed wall/target stage bounds, which remain the final hard bounds.

If the intersection cannot preserve a continuous prefix, fall back to the legacy
Mission profile for that cycle rather than clipping to a discontinuous path.

### Solver handoff

Extended solver failure is a dynamics fallback, not proof that the DP reference is
safe. Once extended mode is in its failure circuit, DP execution authority must be
released unless a current physically validated solved trajectory owns execution.
The legacy 3-state solver may still run, but it receives the bounded Mission fallback
reference rather than the rejected aggressive prefix.

### Closed-loop guard

DP authority also requires bounded lateral/heading disagreement between the measured
state and the first execution stage. This catches nominal wall-safe paths that the
vehicle is no longer tracking. The guard is evaluated before renewing runtime
validation.

## Compatibility

No ROS interface changes. New settings remain in participant YAML and have conservative
defaults. Existing MPCC can be restored by disabling the execution-trust guard for A/B,
but the absolute source-age rule is unconditional.

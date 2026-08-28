# Requirements

## Objective

Remove the contradictory Follow velocity envelope demonstrated by frozen
snapshots 5487 and 5497.  The semantic contract currently computes the
minimum reachable speed with the physical acceleration boundary, while the
seven-state adapter deliberately insets that boundary before optimization.
The resulting state upper bounds cannot be reached by any admissible solver
input.

The first dynamic Gate also froze snapshot 1364: after the input boundary was
unified, its Follow envelope still started at measured-time speed while QP
state zero used delay-compensated control-origin speed.  A reachable temporal
envelope must share both the input boundary and the initial-state timestamp.

## Frozen evidence

- physical braking boundary supplied to Follow: `-3.0 m/s^2`;
- canonical seven-state acceleration lower bound: `-2.9595959596 m/s^2`;
- snapshot 5487 affine minimum common slack: `0.009228407`;
- snapshot 5497 affine minimum common slack: `0.006913547`;
- warm and cold OSQP both reject the immutable problems;
- independently relaxing early acceleration lower rows restores affine
  feasibility.

## Constraints

- Do not change solver tolerances, physical acceleration limits, clearance,
  leases, fallbacks or production authority.
- One physical-boundary inset calculation must own both the Follow reachable
  envelope and the QP input box.
- Do not silently reuse a Cruise artifact as Follow.
- Preserve the policy velocity reference; only the deterministic reachable
  hard envelope may account for the exact executable acceleration bound.

## Acceptance

- The Follow contract receives the same solver-certified braking boundary
  which the adapter installs as the acceleration input lower bound.
- The Follow reachable envelope starts from the exact control-origin velocity
  used by canonical QP state zero.
- A regression using the real physical tolerance proves every generated
  velocity cap is reachable under the assembled input box.
- Existing Follow policy/gap tests continue to pass.
- Build and all package tests pass.

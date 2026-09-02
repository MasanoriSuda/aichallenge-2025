# Requirements

## Objective

Determine whether the rejected instantaneous-gap teacher fails because its
selected steering direction has no executable swept-footprint manoeuvre, while
another short-horizon homotopy is physically available in the same LiDAR
frame.

## Constraints

- Offline audit only; do not connect a new controller to production.
- Keep the packaged base and spatial checkpoints byte-identical.
- Use the kart dimensions, wheelbase, LiDAR offset and published steering
  bounds from repository contracts.
- Evaluate a manoeuvre plus a full-stop contingency, not one polar ray.
- Rebuild every candidate from the current scan; retain no path by timeout or
  lease.
- Compare successful and failed peer teacher bags before generating labels.
- Do not treat a current-scan certificate as a dynamic-obstacle proof.

## Definition of Done

- A pure, tested kinematic rollout and swept-footprint checker exists.
- A sequential replay report distinguishes selected-side, opposite-side and
  no-feasible-candidate frames.
- The same frozen parameters are applied to successful and failed bags.
- The result is admitted or rejected before any dataset/runtime change.


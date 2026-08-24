# Requirements

> Status: rejected after dynamic A/B. The experiment is preserved as root-
> cause evidence; all source and test changes were removed.

## Objective

Make every steering transition stored in a canonical five-state MPCC plan
publishable by the 40 Hz actuator contract. A plan must not be solver-certified
with an inter-stage steering jump which the normal publisher must later reject.

## Failure-first evidence

Bounded run `output/20260824-230215`, Domain 1, crossed the circular seam twice.
After the circular progress representation defect was removed, the two seam
passages still produced six Follow Emergency publications with
`canonical steering continuity rejected: unreachable`:

- plan 4194, stages 2 and 3;
- plan 4202, stage 2;
- plan 7725, stages 2 and 3.

The live publisher permits one control-cycle steering change of 0.0175 rad.
Observed stored-plan jumps were 0.0368--0.0780 rad. Static tracing shows that:

- stage zero is constrained against the previous published steering using one
  model control period;
- later curvature differences are constrained using the MPCC stage duration,
  which may be much larger than one control period;
- canonical execution publishes the selected stage command directly;
- live admission correctly rechecks it against one control period.

Therefore the solver and publisher use different actuation contracts.

## Constraints

- Do not widen steering-rate, solver, wall, gap, timeout, lease, or grace values.
- Do not interpolate or clamp a certified command after solving.
- Keep live steering-continuity admission fail-closed.
- Preserve the existing five-state problem shape and sparse row layout.
- Do not modify or commit `aichallenge/result-summary.json`.

## Definition of Done

- Every canonical five-state inter-stage curvature bound represents one
  publish period, not the longer prediction-stage duration.
- The immutable canonical plan records its initial steering, wheelbase, and
  maximum per-publication steering step.
- Plan validation rejects a stage-zero or inter-stage steering transition
  which is not publishable.
- Unit tests reproduce and reject the previously legal/unpublishable plan.
- Build and full package tests pass.
- A bounded multi-lap `make dev2` run classifies steering-continuity and
  Emergency counts against `output/20260824-230215`.

The attempted repair did not meet dynamic acceptance: constraining every
coarse prediction-stage target to a single 40 Hz publication step made normal
Track/Cruise planning unavailable before the vehicles could start.

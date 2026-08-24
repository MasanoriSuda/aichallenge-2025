# Design

## Root cause

The five-state formulation represents steering as one piecewise-constant
curvature input per prediction stage. At a stage boundary the implementation
publishes the next value directly; it does not publish intermediate ramp
samples. Nevertheless, the inter-stage QP row currently allows

`steer_rate_max * prediction_stage_dt`.

When the prediction stage is longer than the 40 Hz publication period, this
certifies a jump which cannot be issued by the actuator in one publication.
The downstream continuity gate then removes normal authority. This is not a
steering-limit tuning problem: two components interpret the same plan under
different time bases.

## Repair

1. Define a pure conversion from the maximum steering-angle step per
   publication to a conservative curvature-difference bound.
2. Use that single per-publication bound for every canonical five-state
   inter-stage curvature row. Keep row count and sparsity unchanged. The
   separate legacy three-state RTI-SQP path is unchanged and remains removal
   scope for the migration Slice.
3. Extend the immutable canonical plan contract with:
   - wheelbase;
   - steering at the problem snapshot;
   - maximum steering step per publication.
4. Validate stage zero against the snapshot steering and every subsequent
   stage against its predecessor in steering-angle coordinates.
5. Populate those fields from the same extended problem that owned the QP
   first-curvature bound.
6. Keep the current-world live continuity gate. It still protects asynchronous
   delay, missed publications, and supervisor interruptions.

## Dynamic falsification

The proposed repair assumed that a coarse prediction-stage curvature target
could also serve directly as one 40 Hz actuator sample. Run
`output/20260824-232452` falsified that assumption. With every inter-stage row
limited to one publication step, both domains remained at `wp_id=29` and lost
Track/Cruise authority: Domain 1 through QP solve failure and Domain 2 through
the new exact plan-contract rejection.

The coarse MPCC stage must remain free to express the steering evolution over
its full duration. A correct future design must represent that evolution
inside the canonical artifact, for example by making steering angle a state
and steering rate an input, or by solving and certifying a dense actuator-rate
trajectory. Reinterpreting the coarse endpoint as an instantaneous command is
the underlying representation error.

## Why this is structural

The change prevents an internally contradictory artifact from existing rather
than adding a recovery path after rejection. The solver row, immutable plan,
and publisher all share one publication-period definition. No command is
mutated after certification.

## Non-scope

- steering-state/rate-input (six-state) formulation;
- interpolation between coarse prediction stages;
- physical Follow hard-gap failures;
- async delay after a real supervisor authority interruption;
- parameter tuning.

The source implementation described above was removed after falsification. It
must not be reintroduced as a smaller threshold or intent-specific exception.

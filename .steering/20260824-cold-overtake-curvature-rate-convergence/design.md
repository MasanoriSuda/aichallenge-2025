# Design

## Failure chain under audit

`previous_steering` and the stage-zero curvature box are produced separately.
The extended QP then applies both the ordinary input box and a second unary
stage-zero curvature-rate row to the same variable. Cold Overtake solves can
remain outside the latter row until OSQP reaches its iteration limit.

## Hypotheses

1. The input box and steering-rate interval have an empty physical
   intersection.
2. `previous_steering` is not the steering coordinate used by the published
   command.
3. The two intervals overlap, but representing their intersection as two
   active unary rows creates a redundant numerical owner for the same physical
   bound.
4. Row 270 is only a downstream symptom of another constraint/formulation
   defect.

## Investigation sequence

1. Add a pure stage-zero curvature reachability resolver and deterministic
   tests, without changing the QP.
2. Include original input box, steering-rate interval, intersection, previous
   steering, and steering step in solve-failure evidence.
3. Reproduce dynamically and classify the failure as empty-intersection or
   overlapping-duplicate.
4. Only after classification, repair the producer. If hypothesis 3 is proven,
   intersect the steering-rate interval into the existing input box and make
   the duplicate stage-zero rate row inactive while retaining the row layout.
5. Re-run static and bounded dynamic validation.

## Non-goals

- Legacy three-state MPC cleanup.
- Warm-start lineage beyond the canonical paths handled in the preceding
  Slice.
- Performance tuning or racing-line changes.


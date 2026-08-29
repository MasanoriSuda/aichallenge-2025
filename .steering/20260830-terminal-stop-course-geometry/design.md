# Design

## Causal chain

```text
normal artifact publishes a short executable prefix
  -> retained continuation proves only one publisher interval
  -> terminal Stop must cover the remaining braking distance
  -> Stop sampler reaches the end of the executable prefix
  -> last prefix curvature and tactical lateral interval are clamped forever
  -> path-tracking Stop leaves that expired interval
  -> generic exact-trajectory validation reports invalid-lateral-bounds
  -> wall-grid/dynamic proof never runs
  -> normal authority is removed and Emergency Stop owns the wire
```

The visible deceleration is a downstream safety response.  The upstream
defect is that two different horizons share one geometry representation.

## Ownership correction

Keep `ExecutionArtifact` as the executable-prefix owner.  Add a separate
full-horizon terminal course geometry to the immutable physical proof
snapshot:

- local progress knots;
- per-interval course curvature;
- physical lateral lower/upper support at each knot.

The source is the same solver snapshot and physical wall profile already used
to build and certify the fresh plan.  The certified plan retains this physical
snapshot, so retained revalidation consumes exactly the sealed source rather
than reconstructing geometry from Mission state.

`build_stop_contingency()` receives this geometry explicitly.  Sampling past
its last knot returns `course-geometry-unavailable`; no extrapolation is
permitted.  The exact trajectory still uses the physical lateral support, and
the existing footprint wall sweep remains the final physical authority.

## Why this is not a fallback

No alternate command is introduced.  The same terminal Stop already required
by partial-prefix admission is modeled against the correct horizon.  A Stop
which is outside the full physical support or which collides in the exact
wall/dynamic proof remains rejected.

## Verification design

Unit tests must demonstrate all three outcomes:

1. braking proceeds beyond the short executable prefix when full physical
   geometry covers it;
2. missing full-horizon support fails closed instead of extrapolating;
3. violation of the full physical support remains rejected.

Dynamic acceptance compares `output/20260830-011957` with a new `make dev2`
run.  The target signature is a reduction of exact
`invalid-lateral-bounds` terminal failures while preserving wall/dynamic proof
and zero uncertified normal publication.


# Audit

## Root cause addressed by this Slice

The rejected stage-publishability experiment proved that imposing one 40 Hz
steering step on every coarse five-state prediction stage destroys ordinary
Track/Cruise feasibility.  The missing contract is an explicit steering state
and steering-rate input, not another curvature clamp or solver tolerance.

This Slice measures that replacement formulation beside production.  It does
not attempt to suppress a symptom in the five-state controller.

## Semantic-contract audit

- The rate-resolved request is materialized in
  `build_extended_progress_problem()` from the same Track/Cruise references,
  state/input boxes, weights, linear progress reward and stage timing.
- The old first-curvature 40 Hz intersection is intentionally excluded from
  the six-state request.  Observed steering is state zero and steering rate is
  its only reachability owner.
- Track/Cruise shadow eligibility requires `live_progress_active == false`.
  Therefore Overtake-only progress-aligned physical-wall affine rows are not an
  omitted active contract.  The ordinary lateral wall interval remains in the
  copied state boxes.
- The worker receives an immutable decision ID, source problem fingerprint,
  stage-geometry ID, intent, sequence and snapshot time.
- No warm start is transported.  Reusing an unshifted coarse horizon would be
  an invalid provenance claim.

## Authority audit

The result path is:

```text
rate-resolved worker
  -> observation-only Mailbox
  -> aggregate diagnostic counters/log
```

The new types expose no plan-store or authority API.  Source-contract tests
also reject any call from this path to canonical command construction,
solution-history publication, steering state or final control mutation.
Production remains the established five-state canonical Track/Cruise chain.

## Static conclusion

The implementation is eligible for dynamic shadow evidence.  It is not
eligible for production authority, physical-certificate equivalence or warm
start promotion in this Slice.

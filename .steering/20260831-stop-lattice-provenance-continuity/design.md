# Design

## Root cause

`mpcc_rate_resolved_stop_lattice_shadow::evaluate()` copies and rebases the normal source into a Stop candidate, solves that candidate, and builds a certified plan. The final `certified::build()` call currently omits `solver_source_snapshot`, so the accepted plan contains a valid execution artifact and physical proof but no immutable record of the problem which generated it.

Production publication correctly requires:

```text
selected plan
+ matching execution artifact
+ matching solver source snapshot
```

The Stop-lattice plan violates the third term. Once selected, publication rejects its own source and calls `invalidate_published_stop_lattice_observation()`. The bridge therefore becomes one-shot and normal authority chatters.

The downstream external Stop is a symptom of missing provenance, not evidence that the source-validation guard is too strict.

## Rejected alternatives

### Permit a null source for Stop-lattice plans

Rejected. This weakens the immutable problem/solution/certificate join and makes replay or source revalidation impossible.

### Keep the prior alternate after publication rejects its source

Rejected. This retains evidence after the canonical provenance guard says it is incomplete.

### Copy the original normal source pointer into the Stop plan

Rejected. The Stop solver did not solve the original normal request. It solved a publisher-boundary-rebased problem with maximum-braking velocity equality and a complete Stop horizon.

## Selected correction

The Stop-lattice evaluator materializes the rebased Stop candidate as an immutable shared snapshot. The private solver and certified-plan builder use that exact snapshot:

```text
published normal source
  -> publisher-boundary Stop rebase
  -> immutable Stop solver source
  -> fixed-rate private solve
  -> exact trajectory + wall/dynamic proof
  -> CertifiedPlan(execution artifact, proof, exact Stop solver source)
```

This restores an already-established `CertifiedPlan` invariant instead of adding a new lifecycle exception.

## Expected runtime effect

When source 1629's Stop-lattice alternate is selected, final publication sees `source=1` with matching identity. Because the published artifact identity is unchanged, the current alternate remains available for subsequent current-world revalidation instead of being invalidated immediately.

The current-world evaluator still decides every cycle whether the plan is usable. Static wall blockage, dynamic blockage, steering unreachability or intent mismatch remain valid rejection reasons.

## Residual classification

If the same plan remains available but later fails `static-path-blocked`, the next investigation is physical Stop-suffix geometry/time evolution. If provenance continuity removes authority chatter but actual wall contact remains, wall execution mismatch is independently actionable.

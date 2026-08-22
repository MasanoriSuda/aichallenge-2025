# Canonical numerical-boundary contract design

## Causal chain

```text
five-state QP lower bound: virtual progress speed >= 0
-> OSQP returns a finite value slightly below zero within row tolerance
-> persistent solver preserves the signed value for explicit later certification
-> lateral and physical geometry certificates pass
-> CanonicalExecutionPlan requires strict virtual progress speed >= 0
-> canonical extraction rejects an otherwise certified plan
-> fresh canonical authority coverage has periodic holes
```

This is a producer/consumer contract mismatch, not a wall-clearance, solver-load or tuning problem.
The pre-existing test `PreservesFiniteSignedBoundaryValuesForLaterCertification` confirms that the
solver adapter intentionally does not silently clamp numerical boundary values.

## Rejected fixes

### Relax canonical validation

Rejected.  It would let negative semantic speeds enter the immutable execution plan and move the
numerical tolerance ambiguity into every later consumer.

### Clamp the raw solver primal

Rejected.  It would corrupt solver residual provenance and warm-start identity, and would hide an
out-of-tolerance solver result.

### Change OSQP tolerances

Rejected.  The mismatch exists for any nonzero numerical tolerance and tuning would only change
its frequency.

### Add a cycle-local fallback

Rejected.  That recreates the legacy/canonical authority split that this migration is removing.

## Selected design

Add one pure `normalize_extended_execution_primal()` boundary in `mpcc_progress`.

Inputs:

- complete raw five-state primal;
- exact per-row constraint violation and tolerance from the same solve;
- horizon size.

For each state velocity and virtual-progress input, resolve the corresponding identity box row.
The row must be finite and satisfy `violation <= tolerance`.  A negative value additionally must
satisfy `abs(value) <= tolerance`; only then is that value copied as zero into a separate execution
primal.  All other values remain bit-for-bit unchanged.

The Track/Cruise shadow pipeline then has two explicit artifacts:

```text
raw primal        -> residual telemetry and warm start
execution primal  -> actuation, trajectory, physical proof, conversion and canonical plan
```

This makes every executable consumer agree without weakening the canonical plan contract.  The
result records normalization count, maximum adjustment and the first rejected field/stage/value.

## Authority and deletion audit

- No new flag, fallback, timeout or authority is added.
- No production selector or publisher is changed.
- The old implicit assumption that raw numerical primal is already semantic-executable is replaced
  by the typed boundary; no parallel normalization path is retained.
- Retained-plan work does not start in this Slice.

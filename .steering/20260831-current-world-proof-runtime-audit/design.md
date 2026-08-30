# Design: current-world proof runtime audit

## Rejected hypothesis

The first hypothesis was a candidate-clock to published-clock discontinuity.
It is rejected for the frozen decision: both clocks resolve the same 0.840 s
artifact cursor.  The retained evaluator already re-anchors velocity to the
fresh control-origin state.  The logged 1.99 m/s value is old-artifact
diagnostic state, not the serialized speed command.

## Current causal hypothesis

```text
partial current-world proof
  -> terminal Stop contingency required
  -> synchronous retained join takes longer than 25 ms
  -> next callback starts after the certified command interval
  -> current world differs again
  -> terminal suffix can become wall-infeasible
  -> external Stop
```

The hypothesis does not yet identify whether construction, dynamic checking
or wall checking dominates.  Instrument those existing boundaries before
choosing between algorithmic optimization, scheduling isolation or a
different terminal certificate representation.

## Instrumentation

Add a typed runtime breakdown to retained `Result`.  Measure monotonic elapsed
time at existing boundaries; do not create new work.  Copy it into the
controller evaluation and print it only in the existing transition summary.

The sum is diagnostic.  No timer value participates in authority selection.

The selected evaluation alone is insufficient because the production join can
evaluate candidate, published, executed, sibling, Stop-lattice, Gate A and the
previous intent in one callback.  Therefore the audit also records aggregate
plan-evaluation time and the existing production-join subpaths.  This is still
observation-only and does not alter their order or acceptance.

# Design

## Hypotheses

| Hypothesis | Supporting evidence | Falsifier |
|---|---|---|
| The lattice source is too slow | newest source was still running at decision 1494 | an accepted current-source plan already exists and joins |
| A prior accepted plan can bridge the loss | accepted Stop observations existed before Pass authority loss | unchanged current-world evaluation rejects it |
| Existing Stop production source is the only defect | external Stop began after ordinary continuation rejection | lattice plan is absent or also rejects |
| The Stop certificate model disagrees with runtime proof | source-world exact proof accepted | current-world join fails only after a successful continuation solve |

## Observation boundary

The Stop mailbox remains the only producer.  When an accepted result is
consumed, the controller retains its immutable `CertifiedPlan` in a diagnostic
slot.  It grants no authority.

At the first cycle where ordinary ShiftOut/Pass retained evaluation has no
production authority, the observer passes that plan through the existing
current-world plan evaluator using the current problem, time and requested
intent.  Only reason/provenance/timing are recorded.  The returned authority,
if any, is discarded.

Publishing a non-Overtake normal command, external Stop or an override clears
both the worker source and diagnostic plan.  A new Overtake source may replace
the diagnostic plan only after its complete lattice result is accepted.

## Authority invariant

```text
Stop lattice mailbox -> diagnostic plan -> pure current-world evaluation
                                      -> telemetry only

No Store write, no normal adapter, no pending actuation, no publisher edge.
```

## Why this precedes promotion

Source-world certification does not prove that a result is still executable
when normal authority is lost.  Promoting it without this join observation
would turn an asynchronous diagnostic into another stale fallback.  The
classification decides whether the next change belongs in scheduling,
candidate generation, provenance, or the production source edge.

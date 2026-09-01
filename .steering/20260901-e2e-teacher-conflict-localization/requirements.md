# Requirements

## Objective

Localize the teacher/production-normal label conflict to immutable sequences and
samples before changing the training corpus.

## Constraints

- keep the production v11 model and runtime authority frozen;
- do not filter or relabel a sample in this Slice;
- preserve original sequence-local sample indices after deterministic sampling;
- derive normal-distance thresholds only from different admitted normal runs;
- report the frozen focus failure tail independently from aggregate data;
- replay the current teacher semantics and attribute conflicts to decision
  reasons;
- no runtime trigger, threshold, model or authority change.

## Definition of Done

- the observability audit reports per-sequence and causal-tail conflicts;
- exact conflicting samples can be traced to teacher and nearest normal states;
- current teacher reasons are summarized for normal-label disagreement and the
  strongest exact-input conflicts;
- evidence establishes whether conflict-aware data admission is safe to test.

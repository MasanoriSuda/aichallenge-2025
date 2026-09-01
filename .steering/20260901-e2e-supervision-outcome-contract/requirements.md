# Requirements

## Objective

Audit whether each E2E steering label is an executed, outcome-certified
demonstration or only a counterfactual heuristic proposal.

## Constraints

- production v11, runtime authority and every dataset array remain frozen;
- no sample is relabelled or removed in this slice;
- a motion-only recording is not equivalent to Finish with zero penalties;
- source control mode, domain result and immutable source bag identity are
  mandatory evidence;
- successful-normal zero labels and teacher proposals are reported separately;
- generated reports remain under `output/` and are not committed.

## Definition of Done

- the current teacher and normal corpora are audited sequence by sequence;
- missing or contradictory outcome provenance is fail-closed;
- sample-weighted evidence classes are reported;
- a replacement supervision contract is specified before another model is
  trained.

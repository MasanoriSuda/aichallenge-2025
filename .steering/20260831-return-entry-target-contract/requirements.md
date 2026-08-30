# Requirements: Return-entry target contract

## Objective

Classify the first D1 authority loss after a certified Pass in bounded run
`output/20260831-072258`. Determine whether the failure is caused by Mission
lifecycle, Return candidate generation, single-SQP approximation, model/proof
mismatch or physical infeasibility.

Frozen evidence:

- baseline: `041b0fa4`;
- run/domain: `output/20260831-072258/d1`;
- last certified Pass artifact: source sequence `936`;
- first authority loss: decision `1690`;
- snapshot:
  `000000001690-e6ec62c038f0c03b-pass-side-positive-physical-proof-terminal-contingency-unavailable`.

The audit must also inspect the earliest fresh replacement before authority
loss. Sequence `942` at decision `1632` is frozen as
`000000000942-c4f0cb78823e85e5-pass-side-negative-wall-refinement-coupled-solve-rejected`.
It is approximately 1.5 s earlier than decision 1690 and therefore takes
causal precedence over the later Return/Stop symptoms.

## Constraints

- no Mission resume rule, lease, grace, timeout, retry or fallback;
- no solver tolerance, iteration, weight, wall or clearance change;
- no production authority change before same-snapshot classification;
- do not treat the later wall intersection or Recovery as the root cause;
- do not assume Return must retain a target tube after rear/body clearance.

## Definition of Done

- freeze the causal boundary from certified Pass to authority loss;
- run the existing A/B/C/D architecture comparison;
- trace who owns target relation, Return geometry and terminal successor;
- state the earliest violated invariant;
- implement only the repair selected by evidence;
- pass focused/full tests and bounded dynamic validation.

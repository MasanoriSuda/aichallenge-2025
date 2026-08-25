# Design

## Causal boundary

The tactical worker remains responsible for comparing left and right
homotopies. Its trajectory is not suitable for live actuation after Track or
Follow has published intervening commands.

For each newly selected asynchronous plan identity, the prototype made the
live callback perform one observation-only reconstruction:

1. take only the asynchronous selected side;
2. clone the current model, path and V2X world;
3. rebuild current left/right tactical candidates on that isolated snapshot;
4. evaluate the selected side through the existing prospective Overtake
   problem builder;
5. run the existing six-state solver, exact swept-wall proof and target-tube
   proof;
6. log the result and discard the produced plan.

The existing isolated branch evaluator is intentionally reused in this first
shadow Slice. It also computes the tactical five-state evidence, so total time
is a conservative upper bound for a later selected-side six-state-only Gate A.
No result is copied into `overtake_preentry_canonical_plan`.

## Why this is not production promotion

The bounded run proved the live reconstruction is too expensive: successful
and rejected observations cost 88--115 ms against a 25 ms control period.
The synchronous prototype is therefore removed in the same Slice. Promotion
requires a later Slice that extracts a causal, selected-side six-state producer
from the callback, pipelines it like Track/Cruise, and removes the stale-plan
Gate-A route atomically.

## Rejected alternatives

- Relax retained steering/velocity reachability: accepts a plan with the wrong
  predecessor and hides the causal defect.
- Reuse a younger asynchronous result: age alone does not prove predecessor
  continuity.
- Promote the synchronous result immediately: dynamic timing evidence is not
  acceptable because dynamic timing exceeded the callback budget.

# Requirements: Architecture escape-hatch platform

## Objective

Freeze `b6da7ebb296292bf57929ed9064a9fb789b95df0` as the current
seven-state review baseline and make architecture comparisons reproducible
before any further production-controller patch or Slice 7 tuning.

## In scope

- Separate the single-normal-authority invariant from replaceable planning,
  Mission, convexification and solver implementation hypotheses.
- Add explicit Algorithm Pivot triggers to the package policy and root-cause
  audit workflow.
- Define a versioned immutable failure-snapshot manifest.
- Define a central experiment registry with accepted, rejected and
  inconclusive outcomes.
- Provide a dependency-free offline validator and A--D result classifier.
- Add failure-first tests for incomplete evidence and overconfident physical
  infeasibility classification.

## Out of scope

- No production authority, command, controller parameter, clearance, solver
  tolerance, timeout, lease, grace, retry or fallback change.
- No claim that existing logs contain a complete replayable solver snapshot.
- No production adoption of a ManeuverBundle, polynomial/lattice candidate or
  offline nonlinear solver in this Slice.
- No Slice 7 parameter tuning.

## Invariants

1. A single certified normal authority remains mandatory.
2. Mission representation, candidate generator, MPCC formulation,
   convexification and solver backend remain replaceable hypotheses.
3. Snapshots from different commits/configurations/runs are never merged into
   one causal timeline.
4. Every compared method consumes the same immutable world/problem
   fingerprint.
5. `all methods failed` is `unknown` unless an explicit physical
   infeasibility certificate is present.
6. A comparison result cannot alter production authority.

## Definition of done

- Policy, schema, validator/classifier and registry tests pass.
- One manifest representing the current incomplete dynamic evidence is
  recorded as non-replayable rather than silently treated as A--D evidence.
- Existing generated result files remain untouched and uncommitted.
- The next required evidence is explicit: a complete current-world solver and
  physical-proof snapshot from the first Pass/Return failure.

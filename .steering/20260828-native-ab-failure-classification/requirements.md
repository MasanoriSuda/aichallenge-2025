# Requirements: Native A/B failure classification

## Objective

Capture one native schema-v2 Overtake failure from the current HEAD and run
the offline persistent-A versus stateless-B comparison against that one sealed
world/problem fingerprint.

## In scope

- Use the existing observation-only failure recorder and offline comparator.
- Prefer a Pass or Return failure; accept ShiftOut only as secondary evidence.
- Compare A, B-left and B-right with independent instances of the unchanged
  seven-state SQP and the common physical proof.
- Compare the observed failure family with `.steering/ano` without treating the
  upper-participant log as deterministic solver evidence.
- Register the result and update the architecture escape-hatch task state.

## Out of scope

- No production authority or command change.
- No Mission rule, lease, grace, timeout, retry or fallback addition.
- No solver tolerance, clearance, weight, horizon or cadence change.
- No C/D implementation before A/B evidence is classified.
- No mixing of schema-v1 snapshots with schema-v2 replay evidence.

## Definition of done

- A current-HEAD schema-v2 snapshot is replay-ready and fingerprint-valid.
- The offline comparator is run on the exact recorded snapshot.
- The result is classified without overclaiming physical infeasibility.
- A failed or missing observation is reported as inconclusive, not repaired by
  changing production behavior.
- All launched runtime processes are stopped after the bounded observation.


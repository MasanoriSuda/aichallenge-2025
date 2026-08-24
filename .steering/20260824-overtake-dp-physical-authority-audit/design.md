# Design

## Diagnostic contract

Extend the receding-horizon evaluation result with one decisive physical
failure record:

- cause: invalid input, static-map contact, or lateral-acceleration infeasible;
- stage index and path distance;
- evaluated lateral target and physical wall interval;
- profile heading offset;
- required lateral acceleration;
- candidate speed and wall clearance used by that validation attempt;
- whether the record came from the first configured contract or a later repair
  attempt.

The coupled profile search records the candidate that actually failed. The
outer validation loop copies the first configured-contract failure when
available and otherwise the last decisive failure. This prevents a later
aggregate flag from erasing the producer evidence.

## Failure-boundary trace

Immediately before a hard physical failure can invalidate the Mission, emit
one structured warning containing:

- episode, Mission generation, target, side and phase;
- DP active/authority/source age and DP side;
- receding failure cause and stage data;
- stored canonical plan presence, plan id, intent generation and side.

The trace is emitted only for the existing hard failure event. It does not add
a periodic log or alter decisions.

## Tests

- Unit-test the physical-cause string mapping.
- Pin all failure-boundary fields with the existing source-contract test.
- Run focused tests, package tests and build before a bounded dynamic run.

## Behaviour impact

None. No threshold, branch, authority, timer or solver input changes.

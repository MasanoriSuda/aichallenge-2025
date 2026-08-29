# Design: Follow stateless authority deletion

## Authority flow

```text
current target + intent + selected homotopy
  -> immutable current-world Follow side candidate
  -> seven-state SQP
  -> exact wall proof
  -> exact current-world dynamic-obstacle proof
  -> existing certified Store / Gate A
```

The old flow attempted retained persistent Follow geometry first and reached
the stateless population only after rejection. It is deleted rather than kept
as a fallback.

## State ownership

The existing Follow homotopy owner retains only target ID, intent generation
and side sign. It owns no path, corridor, certificate, lease or command. Every
candidate trajectory and proof is rebuilt from the immutable current source.

## Certificate join

The Follow candidate evaluator passes the candidate's current-world snapshot
to the existing exact dynamic proof. A candidate is certified only when:

1. the seven-state SQP solves;
2. exact wall proof accepts;
3. exact dynamic proof is valid and clear; and
4. the certified plan is built from that same artifact fingerprint.

No proof failure falls through to a different normal controller.

## Dynamic result

Run `output/20260829-104728`, D1, confirms the new production flow:

- the first Follow artifact is
  `selected/follow-escape-positive/preferred=1/evaluated=1`;
- the artifact passes exact wall proof and
  `exact_dynamic_final=valid/clear` before Store admission;
- no `persistent-follow` candidate appears in production telemetry.

The run is not an overall race-quality acceptance. At decision 1434, both
current-world candidates were rejected before solve because candidate stage-0
lateral bounds did not contain the measured initial state
(`e_y=1.25536`, bounds `[-3.60609, 0.83409]`). The adapter correctly rejected
this invalid problem and Emergency subsequently owned the command. This is the
next candidate-generation invariant to repair; restoring persistent Mission
geometry would only hide it.

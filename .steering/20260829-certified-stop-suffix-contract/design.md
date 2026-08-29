# Design

## Authority invariant

Production normal control may consume a retained artifact only when the whole
remaining executable horizon is certified. A shorter proof is admissible only
after it is extended with an exact, physically certified contingency suffix.
The repository currently carries only a non-authoritative `ContingencyStopIntent`;
it does not yet carry an exact stop trajectory or wall/opponent certificate.

Therefore this Slice removes partial-horizon authority rather than pretending
that the intent is a certificate.

## Changes

1. The retained current-world evaluator classifies any current-stage-only
   static, dynamic, or nonlinear continuation proof as
   `terminal-contingency-unavailable` and returns no proof.
2. The canonical authority contract requires retained candidates to cover the
   whole remaining horizon, matching the already-required fresh contract.
3. The production adapter independently rejects partial proof construction.
4. Tests freeze the former unsafe case and require rejection.

## Follow-up

If availability becomes the dominant limitation, implement the stop intent as
an exact seven-state braking trajectory and certify its swept footprint and
dynamic-obstacle tube. Only then may bounded prefix authority be reintroduced.

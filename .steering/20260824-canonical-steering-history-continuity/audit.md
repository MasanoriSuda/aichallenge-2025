# Audit

## Observed phenomenon

The first ShiftOut initially publishes a certified retained Overtake command,
then rapidly alternates steering targets. The vehicle continues outward until
the current lateral state is outside the retained wall corridor, canonical
authority becomes unavailable, and Emergency/Stuck processing takes over.

## Causal chain

1. A background worker copies `previous_steering` into its tactical snapshot.
2. Its five-state QP certifies the first curvature relative to that snapshot.
3. Other canonical plans publish while the worker is solving.
4. Current-world retained proof revalidates pose/wall/obstacle provenance but
   not the candidate's steering transition from the latest published command.
5. The stale plan is selected and its exact command bypasses post-solve
   steering-rate mutation by design.
6. Repeated plan replacements create target-steering oscillation.
7. The plant leaves the lateral corridor; Emergency braking is the first
   visible failure but not the producer defect.

## Classification

- Root cause: missing current actuation-history certificate at async canonical
  plan selection.
- Contributing cause: repeated latest-result plan replacement makes the stale
  snapshot mismatch observable as oscillation.
- Detection: retained initial-corridor proof detects the lateral departure only
  after it has happened.
- Amplification: Emergency retains the last outward steering while braking;
  Stuck Recovery starts later.
- Falsified: corridor cursor/time-origin mismatch.
- Falsified for this Slice: steering gain tuning as the primary cause.

## First dynamic falsification

Run `output/20260824-203305` proved that selection-time rejection worked but
also exposed an incomplete entry gate:

- `Idle -> ShiftOut` occurred at `1787571227.919`.
- The stored pre-entry plan was then rejected as unreachable from the live
  steering (`current=-0.117915`, `candidate=-0.047970`, maximum step
  `0.0175 rad`).
- Because Overtake authority had already been raised, the only remaining owner
  was `canonical-normal-emergency-stop`.

This is not evidence that the continuity limit is too strict. It proves Gate A
admitted semantic/physical evidence without live actuation evidence. The
repair therefore extends the same pure contract into new-entry and active
replacement admission and moves new-entry admission before Mission mutation.

## Retained progress evidence

Run `output/20260824-214343` added exact rejection evidence before changing the
contract:

- decision 1121 rejected a `-2.94803e-8 m` progress delta even though the
  immutable solution reported `maximum_constraint_violation=6.10639e-5`;
- decision 1138 rejected `-4.20999e-9 m` against a `2.05136e-6` certificate;
- the rejected fresh result forced reuse of an older plan until its steering
  history was no longer reachable.

This falsified real reverse progress as the source.  The downstream fixed
`1e-9 m` rule contradicted the solver certificate and made a valid fresh plan
unavailable.

## Validation

- `make autoware-build`: 25 packages succeeded.
- Correct workspace test tree: 40/40 test targets, 1795 tests, 0 failures.
- `make dev2`: `output/20260824-215632`.
  - `invalid-progress-evolution`: 0 occurrences on Domain 1 and Domain 2;
  - a certified canonical candidate that exceeded current steering
    reachability was rejected instead of being published;
  - Overtake pre-entry reached `ShiftOut` without a pre-entry steering-gate
    rejection after phase mutation.

The run also identified a separate next-Slice defect.  At
decision 1464 Cruise published `-0.32 rad`; the behavior then changed back to
Follow while the first new Follow result was still pending.  The only retained
Follow artifact was plan 1462 at stage 4 (`-0.19 rad`) and was correctly
rejected as unreachable.  The resulting Emergency is therefore no longer a
hidden steering discontinuity: it is an atomic intent-admission defect at the
Cruise/Follow boundary.  It must be repaired by keeping the currently
certified intent owner until the requested intent has a live canonical plan,
not by weakening steering continuity or adding a fallback.

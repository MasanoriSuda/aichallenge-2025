# Design

## Root cause

`lift_progress_to_retained_branch()` already computes the integer lap offset
which places a measured progress nearest to retained-plan progress. Before
that calculation, however, it imposes a second and narrower representation
contract: circular measured progress must be in `[0, path_length)`.

The projection layer can legitimately represent the seam as `path_length`
and can transiently expose a small negative equivalent coordinate. These are
the same physical points as `0` modulo the path length. Consequently a valid
current pose is rejected before the function reaches the branch-lift and
continuity checks.

The visible Emergency and subsequent steering-continuity rejection are
downstream consequences of losing every canonical normal candidate at the
seam.

## Repair

1. Keep finite input, positive path length, and unambiguous tolerance checks.
2. Remove the artificial `[0, path_length)` representation restriction.
3. Apply the existing nearest-integer lift directly to any finite circular
   coordinate:

   `lifted = measured + round((retained - measured) / length) * length`

4. Preserve the existing absolute continuity proof on the lifted result.
5. Add deterministic regression cases for the exact endpoint, a negative seam
   coordinate, and a multi-lap coordinate, plus a discontinuous counterexample.

## Why this is structural

The repair defines one coordinate equivalence contract at the shared retained
proof boundary. It does not special-case a waypoint, intent, target, or log
event, and it removes a conflicting downstream assumption about how the
projection layer must encode a circular seam.

## Non-scope

- physical hard-gap violations;
- async plan cursor progression after a real authority loss;
- steering-rate tuning;
- Overtake tactical planning;
- parameter tuning.

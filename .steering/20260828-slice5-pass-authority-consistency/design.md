# Design

## Root cause

The target-bound execution hold has two separate responsibilities: retaining a
certified lateral prefix while a replacement is solved, and retaining the
speed at which that prefix began. The latter was applied unconditionally after
the committed-Pass front-cap policy. Consequently a current-world prediction
could revoke closing-speed authority, then an older Mission-lifecycle patch
could restore it before publication.

This is an authority-ordering defect, not a clearance or solver-tuning defect.

## Repair

Introduce one pure resolver in `v2x_overtake_core` for target-bound replan speed
retention. It receives the current-world dynamic certificate and the already
resolved speed values. It may raise the reference/floor only when all are true:

- a target-bound prefix is executing;
- the front cap is released;
- pre-contact squeeze escape is inactive;
- current footprints are separated;
- the prediction is valid;
- the predicted footprint sweep is separated.

The physical prefix may remain the lateral authority when this speed retention
is revoked. This keeps trajectory continuity without allowing stale
longitudinal authority to defeat a current-world risk decision.

## Diagnostic repair

Change execution-artifact construction from a bare `optional` to a result that
also carries its validation/construction reason. Post-refinement proof must not
turn an artifact build failure into a default physical-proof failure.

## Non-goals

- No new timeout, lease, fallback, or Mission lifecycle state.
- No parameter change.
- No production-authority expansion.
- No attempt to make an infeasible Pass feasible by weakening proof.

# Requirements

## Objective

Guarantee that the first constant steering-rate stage remains physically
reachable from the immutable semantic current steering, even when the solver's
state-zero equality is satisfied only within its certified numerical
tolerance.

## Root cause

After duplicate downstream checks were removed, run
`output/20260825-011102` exposed 19 real 25 ms steering-limit violations. The
QP derives first-stage dynamics from a solver variable for state zero. That
variable may be slightly inside the semantic actuator state under the accepted
equality tolerance, so the generic rate box can permit an outward rate that is
not reachable from the actual steering value.

## Scope

- Expose the persistent solver's physical absolute/relative row-tolerance
  contract without changing it.
- Intersect only first-stage steering rate with the exact reachability interval
  derived from semantic current steering and the whole first-stage duration.
- Move that interval inward by a mathematically sufficient solver-certificate
  margin.
- Record physical/solver bounds and margin in shadow telemetry.
- Add deterministic boundary and infeasibility tests.

## Non-scope

- No clamp, solver setting, physical limit or parameter change.
- No later-stage rate-policy change.
- No production authority or fallback change.
- No cross-stage publication sampling repair.

## Preserved user state

`aichallenge/result-summary.json` is a pre-existing user change and must not be
edited, staged or committed.

## Rollback

Rollback target: `f06655a`.

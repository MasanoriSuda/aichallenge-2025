# Requirements

## Purpose

Remove the unreachable retained DynamicEscape execution path left after the
legacy normal solve producer was physically deleted, while preserving the live
fresh DynamicEscape candidate and its current-cycle physical wall admission.

## Earliest violated invariant

A retained execution may influence publication only if a reachable producer
first creates a complete, same-identity execution artifact. The controller has
no assignment to `pending_dynamic_escape_execution_`; therefore
`retained_dynamic_escape_execution_` can never be born. The associated
formulation lease is only reset to negative infinity and can never become
active.

## Scope

- Delete the pending/retained DynamicEscape structs, stores, cursor, lease and
  restore/promote helpers.
- Delete retained-only exit-gate state, reasons and tests.
- Reduce DynamicEscape execution ownership to its live invariant: current
  fresh execution, current canonical permission and current physical proof.
- Preserve live DynamicEscape planning, obstacle/wall constraints, final wall
  admission, replan, Emergency and Recovery.

## Non-scope

- Do not delete fresh DynamicEscape or tune its thresholds.
- Do not delete the canonical Overtake ownership boundary or generic wall
  admission gates in this Slice.
- Do not add a replacement lease, fallback, timeout, feature flag or clamp.
- Do not modify or stage `aichallenge/result-summary.json`.

## Definition of Done

- Failure-first source contracts prohibit restoration of the unreachable
  retained store and lease.
- Fresh DynamicEscape remains executable and physically wall-checked.
- No final-publisher branch can claim to publish a retained DynamicEscape
  stage which no producer can create.
- Focused tests, full package tests and `make autoware-build` pass.

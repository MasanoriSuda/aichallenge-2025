# Track/Cruise certificate provenance

## Baseline

- Branch: `develop_july`
- Baseline commit: `699faeb`
- Preserved unrelated change: `aichallenge/result-summary.json`

## Problem

Track/Cruise five-state shadow solutions pass the QP bounds but 2.12% of cycles fail the final
physical wall certificate. The log only reports `hard wall contact` or a swept-path index, so it
cannot distinguish a centre-coordinate bound defect, vehicle-heading footprint expansion, map
sampling failure or an unsafe transition from the current pose.

## Scope

- Define a typed physical-wall certificate diagnostic.
- Record the first failing stage and exact geometry/bound evidence without changing the boolean
  certificate result.
- Add the diagnostic to status-change logging only; do not add per-cycle logging.
- Repeat a single-car run and classify the wall failures by structured evidence.

## Non-scope

- No controller-authority promotion.
- No wall clearance, cost, bound, horizon or solver parameter change.
- No suppression of wall/contact rejection.
- No command behavior change.

## Acceptance

- Deterministic tests fix the diagnostic reason names and mandatory evidence format.
- Existing wall validation call sites retain the same pass/fail behavior.
- Repeated simulation identifies failing stage, waypoint, lateral bound reserve, heading and pose.
- All shadow output remains `authority=shadow, selected=0`.

## Rollback

Rollback commit: `699faeb`.

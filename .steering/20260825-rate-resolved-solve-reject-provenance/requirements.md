# Requirements

## Objective

Preserve the exact typed cause of every non-solved rate-resolved shadow result
until it is emitted by the 2-second aggregate log.

## Evidence

`output/20260825-015302` contained one `SolveRejected` among 6,164 consumed
results. `SolverContext::Result::detail` already carried persistent-OSQP stage,
status, residual and row diagnostics, but the aggregate stored only the latest
result. A later successful result overwrote the failure before logging.

## Scope

- Keep the ordinary latest result telemetry unchanged.
- Separately retain the latest non-solved result in each aggregate window.
- Emit its immutable identity, outcome, solve telemetry and detail.
- Add source-contract coverage that this remains observation-only.

## Non-scope

- No retry, fallback, solver-setting or parameter change.
- No suppression or reclassification of a solver reject.
- No authority promotion.

## Preserved user state

`aichallenge/result-summary.json` is user-owned and excluded.

## Rollback

Rollback target: `60ef8b9`.

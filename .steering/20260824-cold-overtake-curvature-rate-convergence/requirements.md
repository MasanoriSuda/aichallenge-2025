# Requirements

## Objective

Cold Overtake MPCC branch solves repeatedly reach the OSQP iteration limit with
extended constraint row 270 (stage-zero curvature-rate) as the worst physical
residual. Identify the producing defect and repair it without solver tuning,
new fallback authority, or another scenario-specific bypass.

## Evidence boundary

- Baseline: `e98dede`
- Dynamic evidence: `output/20260824-172712/d1/autoware.log`
- Repeated failing solves are cold (`warm_candidate=0`).
- For `N=20`, row 270 is the stage-zero `CurvatureRate` row.

## Constraints

- Do not change OSQP iteration/tolerance/rho settings.
- Do not tune wall, steering, velocity, or cost parameters.
- Do not add retry, cooldown, fallback, lease, or authority paths.
- Preserve the physical steering-rate limit and published-command contract.
- Preserve the extended QP/warm-start row layout unless evidence requires a
  versioned migration.
- Do not touch `aichallenge/result-summary.json`.

## Definition of done

- Stage-zero curvature reachability is independently observable and tested.
- The root cause is demonstrated, not inferred from the row number alone.
- The producer owns one physically equivalent stage-zero curvature bound.
- Focused and package tests pass.
- A bounded `make dev2` run shows whether cold row-270 failures are removed or
  exposes the next upstream constraint.


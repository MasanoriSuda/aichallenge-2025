# Requirements

## Objective

Determine whether six-state pre-entry adoption failures are legitimate stale-plan
rejections or a shared error in the asynchronous time/progress/actuation join.

## Requirements

- Do not change authority, solver inputs, fallback behavior, thresholds or ROS
  interfaces.
- Preserve the existing current-world revalidation decision.
- Emit enough evidence to reproduce each rejection from one decision record:
  artifact age and cursor, measured/expected progress, steering reachability and
  velocity reachability.
- Reuse the retained revalidator's computed values rather than recomputing a
  second diagnostic interpretation in the controller.
- Keep `aichallenge/result-summary.json` untouched.

## Definition of Done

- Failure results retain the decisive numeric evidence even when no proof is
  produced.
- Pre-entry adoption telemetry reports that evidence without changing behavior.
- Unit/source-contract tests pass.
- A bounded `make dev2` run distinguishes legitimate divergence from a broken
  time/progress contract.

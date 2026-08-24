# Audit

## Observed symptom

The accepted piecewise-publication run `output/20260825-015302` consumed 6,164
rate-resolved Track/Cruise shadow results. One result was `SolveRejected`, but
the periodic log showed only the newest successful result and therefore did
not contain the rejected solve's typed detail.

## Root cause

`SolverContext::evaluate()` already preserved the persistent-OSQP failure
detail in `Result::detail`. The loss happened later in the controller telemetry
window: every consumed result replaced `last_result`, so a success arriving
before the next two-second report erased the failure provenance.

This was an observation defect, not evidence that retry, fallback, solver
tuning, or a new guard was needed.

## Change-to-cause mapping

- Keep `last_result` unchanged for the ordinary newest-result summary.
- Keep `last_failure_result` independently whenever an outcome is not
  `Solved`.
- Emit the failure's immutable identity, typed outcome, OSQP iteration/status,
  setup/update/reset flags, and existing detail before resetting the window.
- Keep the trace explicitly marked `authority=shadow, selected=0`.

No command path reads the retained failure and no production authority or
numerical behavior changed.

## Remaining uncertainty

The diagnostic change explains why the cause was hidden; it does not yet
classify the one solver rejection. A dev2 run must exercise another rejection.
If none occurs, the diagnostic is valid but the failure cause remains `NOT
EXERCISED` and no behavioral repair is justified.

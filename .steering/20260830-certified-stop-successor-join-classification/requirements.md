# Requirements

## Objective

Classify the steering-state owner at the certified Stop successor join without
changing production authority, and replace per-cycle observation logs with a
bounded one-second summary.

The first dynamic run, `output/20260830-142647`, produced 1,282 successful
successor samples. Position, yaw and speed generally joined, while steering
error repeated in control-period-sized steps. The current observation only
compares against the command-control-origin steering, so it cannot distinguish
a proof/publisher mismatch from a steering-state semantic mismatch.

## Constraints

- Do not select or publish the certified Stop successor.
- Do not change Mission lifecycle, authority, solver, fallback, timeout,
  clearance, or vehicle parameters.
- Compare one expected successor steering state against all existing steering
  owners at the same current-world evaluation.
- Do not log every control cycle.
- Preserve exact evidence identity and the one-shot next-origin join.

## Definition of Done

- The observation result distinguishes command-control-origin, current-time
  physical, response-control-origin, and previous-published steering errors.
- Missing optional steering axes do not invalidate an otherwise valid
  pose/speed/command-origin observation.
- Logs report counts and error aggregates no more than once per second.
- The previous per-cycle INFO/WARN stream is removed.
- Unit and source-contract tests pass.
- A new `make dev2` run identifies which steering owner, if any, joins the
  certified successor closely enough for a later architecture decision.

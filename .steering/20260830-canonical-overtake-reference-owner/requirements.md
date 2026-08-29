# Requirements

## Frozen evidence

- Baseline: `output/20260830-022414/d1/autoware.log`
- Frozen failure: decisions 1388 through 1405, ShiftOut episode 1.
- The production seven-state artifact `sequence=789` was accepted and published
  at decision 1397.
- The synchronous callback simultaneously spent 23.268 ms in the legacy
  OvertakeLine receding optimiser even though that result was demoted to
  `canonical-reference-only`.
- Control callback execution then exceeded the 25 ms budget repeatedly
  (38.328, 56.042, 64.597 and 48.670 ms).
- Retained current-world proof subsequently failed first as
  `terminal-contingency-unavailable`, then as `steering-unreachable`; the
  emergency command changed the physical successor and the Mission entered
  Recovery.

## Objective

Remove the obsolete synchronous legacy receding-optimisation edge from the
canonical seven-state problem assembly. OvertakeLine remains responsible for
constructing the tactical reference and physical corridor, while the
production seven-state MPCC remains the only optimiser and normal command
authority.

## Constraints

- Do not change solver tolerances, weights, wall clearance or speed policy.
- Do not add a fallback, lease, timeout, grace period or Mission resume rule.
- Do not weaken current-world wall, dynamic-obstacle, actuation or terminal
  contingency proofs.
- Preserve ShiftOut/Pass/Return identity and no-return semantics.
- Keep the baseline horizon evaluator because it constructs the reference and
  stage-wise corridor consumed by the canonical MPCC.
- Delete the call edge to `optimize_live_overtake_line_horizon`; a demoted
  optimiser must not run synchronously merely for observation.

## Definition of Done

- Canonical Overtake problem assembly performs one tactical reference/corridor
  construction and no legacy receding optimisation.
- Static architecture tests prevent the removed call edge from returning.
- Package build and all tests pass.
- In `make dev2`, the 20 ms-class `OvertakeLine runtime ownership` spike caused
  by `receding=1` is absent; any remaining authority failure is classified
  independently rather than hidden by that overrun.


# Requirements: production physical diagonal separation

## Objective

Repair the demonstrated upstream defect in the production rate-resolved MPCC:
the dynamic-obstacle convexifier can express only axis-aligned
behind/side/ahead separation even though a physically certified diagonal
topology exists.

The repair must use the current observation epoch's physical ego footprint and
target radius, retain the exact nonlinear wall/dynamic/successor certificates,
and delete the wall-only `partial_side_escape` mask in the same Slice.

## Frozen evidence

- Rollback commit: `0bc2b8a2`
- Source run: `output/20260828-094214`, Domain 1
- Decision: `1566`
- Interaction fingerprint: `7246006054995400977`
- Certified physical candidate fingerprint: `16820872117393555423`
- Certified physical schedule: diagonal start stage 1, full-side stage 3

## Constraints

- Do not tune wall clearance, vehicle clearance, solver tolerance or OSQP.
- Do not add a fallback, retry, timeout, lease, grace period or resume rule.
- Do not hard-code the frozen `1 -> 3` schedule as production policy.
- Do not allow a physical support half-space to replace exact nonlinear proof.
- Do not retain `partial_side_escape` as a fallback.
- Do not change ROS, launch, Domain, evaluation or submission interfaces.
- Keep production authority on the current seven-state certified pipeline.

## Definition of done

- Every production solver entry receives one immutable physical-world payload
  tied to the same problem/world fingerprint as its dynamic stage predictions.
- The production topology builder derives a diagonal transition without an
  experiment-only forced schedule.
- The frozen failure produces a certified production-equivalent bundle.
- `partial_side_escape` and its telemetry/documented contract are removed.
- Exact wall, timed all-obstacle dynamic and terminal-successor proof remain
  mandatory before publication.
- Focused tests, full package tests and `make autoware-build` pass.
- A later dynamic Gate demonstrates Pass -> Return, or the Slice remains
  explicitly non-promoted.

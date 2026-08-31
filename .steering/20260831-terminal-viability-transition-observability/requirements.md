# Requirements: terminal viability transition observability

## Root-cause gap

The decision-1838 frozen comparison proves that every represented candidate is
infeasible at the first rejected decision.  Runtime logs prove that normal
authority was accepted one control cycle earlier, but they do not retain the
accepted current-world proof fields.  The viable-to-infeasible transition
therefore cannot be assigned to plant/model divergence, course-frame change or
artifact/publication identity.

## Objective

Record the last accepted recursive terminal proof together with the first
failure for the same source artifact and intent.

## Constraints

- Observation only: no authority, solver, fallback or state-transition change.
- No new timeout, grace, lease, tolerance, clearance or parameter.
- Pair only identical source sequence and control intent.
- Do not perform file I/O on the control callback.

## Definition of Done

- The first failure log identifies both decisions and source identity.
- It reports state, steering, speed and terminal-wall deltas across the pair.
- The immutable failure snapshot detail carries the same accepted-boundary
  provenance for offline review.
- Build and focused contract tests pass.


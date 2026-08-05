# Requirements

## Purpose

Prevent a committed outside overtake from silently becoming an inside pass at
the next curvature reversal, and stop the final global mission ranking from
discarding path-clearance information.

## Scope

- Detect an outside-role reversal while evaluating the complete candidate.
- Store a deterministic Pass-relative transition window in the selected
  mission.
- During Pass, request the already existing atomic opposite-side corridor
  replacement when that committed window opens.
- Keep the rolling 12 m detector as a fallback for curvature changes that were
  not visible at admission.
- Carry full-path wall/corridor/Return clearance into final mission ranking.
- Add INFO diagnostics for scheduled transition acceptance and rejection.

## Constraints

- Keep the deferred rear-clear extension introduced by `d3807f3`.
- Do not add `OvertakeArmed` or change acceleration/speed parameters here.
- Do not change ROS topics, messages, launch contracts, or evaluation schema.
- Preserve the user's existing `aichallenge/result-summary.json` change.

## Definition of Done

- A schedulable outside-role reversal is represented by the frozen mission.
- The scheduled transition is attempted before its committed deadline rather
  than relying only on the rolling lookahead.
- An unschedulable transition candidate is rejected before ShiftOut.
- A materially wider full path wins over a narrow candidate without defeating
  the existing racing-progress score for small clearance differences.
- Focused tests and the package build/test pass.


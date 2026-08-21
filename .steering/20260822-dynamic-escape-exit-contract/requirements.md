# Requirements

## Purpose

Prevent Dynamic Obstacle Escape from returning lateral authority to the racing
line while the newly selected control horizon still predicts a wall violation.

## Scope

- Keep the last feasible connected Dynamic Escape execution for a short,
  bounded handoff interval.
- Keep the progress-contouring formulation active through that interval so the
  controller does not switch formulation on the same cycle as lateral authority.
- Physically validate both an active Dynamic Escape path and its outgoing path.
- Request a new Dynamic Escape branch when the executed path is wall-invalid.
- Emit change-aware logs that identify the exit, wall admission result, retained
  solution use, age, and replan outcome.

## Constraints

- Do not change topic, service, launch, evaluation, or result JSON contracts.
- Do not loosen physical wall or vehicle-footprint checks.
- Do not tune racing performance parameters in this change.
- Preserve the user's existing `aichallenge/result-summary.json` modification.

## Definition of Done

- A Dynamic Escape exit requires two consecutive wall-valid outgoing horizons.
- A wall-invalid exit cannot immediately publish the racing-line solution.
- A recent feasible Dynamic Escape solution is used only for a bounded interval;
  otherwise the existing decelerating wall hold remains in force.
- An active wall-invalid Dynamic Escape invalidates the exact target/side and
  permits the alternate branch to be evaluated on the next cycle.
- Unit tests and package build pass.

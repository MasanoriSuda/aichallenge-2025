# Requirements

## Objective

Repair the fresh Overtake entry proof join so a selected and exact five-state
MPCC branch is admitted using its own velocity trajectory, not the preceding
geometric Mission rollout.

## Root cause

The asynchronous dual branch stores an exact physical execution trajectory,
including velocity at every stage. Final progressive-prefix admission ignores
that selected evidence and reuses `predicted_minimum_ego_speed_mps` from the
kinematic Mission candidate. This mixes formulations at the production join
and produced an empty observed entry window in `output/20260824-194340`.

## Invariants

- Keep the 8 m completion-proof reserve unchanged.
- Keep target, wall, body-clear, freshness and current-world certificate
  validation unchanged.
- Exact five-state evidence may take precedence only when the physical
  execution certificate is complete and valid.
- Legacy/geometric producers without such a certificate retain the current
  fail-closed minimum-speed behavior.
- Do not change controller weights, solver settings, cadence or race margins.
- Do not restore a legacy normal authority.

## Definition of done

- Minimum-speed admission reports whether legacy or certified-execution speed
  evidence owned the comparison.
- A complete exact velocity sequence takes precedence over legacy prediction.
- Missing, incomplete or invalid exact evidence falls back to legacy prediction.
- Static tests cover both paths and malformed evidence.
- A clean `make dev2` shows either a certified `Idle -> ShiftOut` before the
  8 m gate or a different, explicitly traced earliest blocker.

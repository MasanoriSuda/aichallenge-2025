# Track/Cruise retained revalidation requirements

## Purpose

Implement Gate B for the canonical five-state Track/Cruise path: when a fresh solve is
temporarily unavailable, a remaining segment of the last canonical plan may become a
**shadow candidate only after it is re-proven against the current observation**.

## Baseline and preserved state

- Branch: `develop_july`
- Baseline: `87514ac`
- Design source: `.steering/20260822-track-cruise-retained-revalidation-design/`
- Preserve the user's existing `aichallenge/result-summary.json` change.

## Required invariants

1. Retained execution uses the same `VelocityProgress5State` formulation as the fresh plan.
2. Elapsed time selects the exact remaining control stage; a partially elapsed first stage
   contributes only its remaining duration and ends at state `k + 1`.
3. Measured circular progress is explicitly lifted onto the retained unwrapped progress branch.
4. A current proof binds decision, intent, ego observation, intent generation, stage geometry,
   target/obstacle generation, control pose, course-frame window and obstacle tube identities.
5. Static wall checks are aligned by absolute progress and dynamic obstacle checks by current-
   observation-relative time plus progress.
6. The measured-to-control delay prefix and the control-pose-to-first-endpoint connector are
   independently checked; no uncovered spatial gap is allowed.
7. Candidate construction accepts only an intact proof fingerprint for the exact plan, cursor
   and current provenance.
8. Missing or mismatched input fails closed and cannot fall back to legacy MPC.

## Non-scope

- No final publisher or normal-authority promotion.
- No Follow/Overtake authority integration.
- No solver, wall-margin, horizon, cost or speed tuning.
- No retry, lease, grace period, fallback mode or configuration flag.
- No weakening of the canonical semantic residual boundary.

## Acceptance

- Failure-first tests cover stage aliasing, partial first stage, provenance mismatches, connector
  and delay-prefix collision, moving obstacle collision, circular seam and missing inputs.
- Focused and full package tests pass.
- `make autoware-build` passes.
- Retained production authority remains disconnected.
- If runtime shadow is connected, telemetry distinguishes unavailable, rejected and accepted
  retained proofs without changing the published control command.

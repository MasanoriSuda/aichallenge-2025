# Phase 0: MPCC single-authority audit requirements

## Baseline

- Branch: `develop_july`
- Baseline commit: `dc51093` (`Preserve dynamic escape execution ownership`)
- Primary runtime evidence: `output/20260822-105057/d1/autoware.log`
- Existing unrelated working-tree change: `aichallenge/result-summary.json`

The baseline commit is the comparison point for this audit. The existing result summary change is
out of scope and must not be modified or committed by this steering.

## Objective

Define an evidence-backed path from the current mixed MPC/MPCC controller to a simpler architecture
in which one canonical MPCC formulation owns normal racing commands.

"Single authority" means:

- Track, Cruise, Follow, Hold, Stop, ShiftOut, Pass, Return, and Rejoin are expressed as intents and
  constraints of one canonical MPCC formulation.
- Lateral and longitudinal commands come from the same certified prediction.
- Emergency stop may override the MPCC command.
- Stuck/gear/reverse recovery remains a separate supervisor because it is outside the forward-racing
  model.
- A failed solve may use a short, bounded last-certified solution from the same formulation, followed
  by emergency stop. It must not switch to another normal-driving controller in the same cycle.

## Scope

This steering shall:

1. Describe the current command-authority graph from observation to publish.
2. Inventory the major guard, fallback, lease, direct-control, and mode-switch patch families.
3. Define machine-checkable invariants for the target architecture.
4. Separate root causes, contributing causes, masks, detection gaps, and recovery behavior.
5. Divide migration into independently reviewable vertical slices with explicit deletion gates.
6. Define deterministic replay and runtime acceptance criteria before implementation.
7. Add package-local engineering rules and a reusable root-cause audit skill.

## Out of scope

This steering shall not:

- change production C++ or Python code;
- change `config.yaml` or `config_for_cloud.yaml`;
- change test expectations;
- tune OSQP, cost weights, margins, timeouts, hysteresis, cooldowns, or rates;
- add a fallback, retry, hold, clamp, grace period, or runtime feature flag;
- enable always-on MPCC merely by setting `progress_contouring_mpcc_overtake_only: false`;
- claim that simulation evidence validates a real vehicle;
- remove legacy control before its replacement passes the slice exit criteria.

## Evidence rules

- A code claim requires `file:line` evidence.
- A history claim requires a commit or `git log -S/-G` result.
- A runtime claim requires a run ID and log/replay evidence.
- Unsupported conclusions must be marked `Unknown` or `Hypothesis`.
- The first violated invariant is more important than the final error message.
- A downstream fallback that delays or changes the symptom is a mask, not a root cause.

## Definition of Done

- [x] Current authority graph documented.
- [x] Major patch families and deletion gates documented.
- [x] Target invariants and current evidence status documented.
- [x] Migration slices and rollback boundaries documented.
- [x] Validation/replay plan documented.
- [x] Package-local `AGENTS.md` created.
- [x] `mpcc-root-cause-auditor` local skill created and validated.
- [x] No production/config/test-expectation change in the Phase 0 commit.

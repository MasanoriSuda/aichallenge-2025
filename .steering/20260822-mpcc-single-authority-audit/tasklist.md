# Phase 0 task list

## Audit

- [x] Freeze the comparison baseline at `dc51093`.
- [x] Preserve the unrelated `aichallenge/result-summary.json` change.
- [x] Read the current MPC integration and interface contracts.
- [x] Trace observation-to-command authority in the current controller.
- [x] Confirm the active solver/formulation fallback chain.
- [x] Inspect representative introduction commits with `git log -S` and recent history.
- [x] Correlate the code structure with `output/20260822-105057/d1/autoware.log`.

## Governance artifacts

- [x] Write `requirements.md`.
- [x] Write `design.md`.
- [x] Write `authority-graph.md`.
- [x] Write `patch-ledger.md`.
- [x] Write `invariants.md`.
- [x] Write `failure-causal-trees.md`.
- [x] Write `migration-slices.md`.
- [x] Write `validation-plan.md`.
- [x] Add package-local `AGENTS.md`.
- [x] Add and validate `mpcc-root-cause-auditor`.
- [x] Register the skill in the repository skill catalog.

## Phase 0 acceptance

- [x] No production source changed.
- [x] No runtime configuration changed.
- [x] No test expectation changed.
- [x] Claims are separated into Confirmed, Hypothesis, and Unknown.
- [x] Every migration slice names branches that should be deleted.
- [x] Parameter tuning is placed after structural convergence.

## Next approval gate

Do not begin Phase 1 automatically. The user must approve one migration slice after reviewing this
audit. The recommended first implementation slice is Slice 1: immutable problem context,
certificate, and final-decision contracts without changing runtime behavior.

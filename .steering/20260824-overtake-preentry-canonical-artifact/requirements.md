# Requirements

## Evidence boundary

- Branch: `develop_july`.
- Baseline: `259804db9f55b0b90599c84a20bd433a4713d499`.
- Runtime evidence: `output/20260824-092036`, Domain 1.
- Existing unrelated user change: `aichallenge/result-summary.json`; do not edit or commit it.

## Objective

Make a new Overtake entry atomic with the exact five-state MPCC artifact which already won the
pre-entry left/right comparison. `Idle -> ShiftOut/Pass` must not occur and then wait for a second,
duplicate canonical solve to produce the first executable plan.

## Repaired invariant

When a dual-MPCC branch authorizes a new entry, the same solved, finite, constraint-valid and
physically certified solution must provide:

1. the selected Mission and side;
2. the immutable canonical execution plan;
3. the initial Overtake canonical lifecycle identity and retained-plan store.

The plan must match prospective intent, Mission generation, target and execution side. Missing or
mismatched artifacts keep the current Follow/Cruise path; they may not enter ShiftOut.

## Scope

- Preserve the canonical plan made from the already solved pre-entry branch.
- Carry it through the existing tactical latest-only result with the selected Mission.
- Validate and adopt it before the live FSM phase transition.
- Let the production Overtake worker refresh that same canonical lifecycle after entry.
- Add deterministic plan-admission and source-contract tests.

## Non-scope

- No solver, weight, clearance, wall margin, timeout, cadence or parameter change.
- No grace, hold, retry, lease, alternate normal controller or synchronous callback solve.
- No Rejoin production promotion.
- No Pass/Return quality tuning.

## Acceptance

- A missing, invalid, wrong-intent, wrong-generation, wrong-target, wrong-side or expired plan is
  rejected before phase transition.
- A matching plan initializes the Overtake canonical context and store before
  `transition_overtake_line_phase()`.
- No second solve is added to the 40 Hz callback.
- Focused tests, package tests and `make autoware-build` pass.
- Bounded `make dev2` evidence shows the first exercised ShiftOut/Pass decision is retained/fresh
  canonical or an explicit physical reject, not `async-pending` caused solely by producer startup.

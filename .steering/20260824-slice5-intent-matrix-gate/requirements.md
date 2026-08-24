# Requirements

## Objective

Complete the remaining Slice 5 dynamic intent matrix on the canonical-only
baseline. Exercise Pass and Return and classify DynamicWait/DynamicEscape
without changing race parameters or restoring a legacy normal authority.

## Baseline

- Source commit: `e20f880`.
- Track/Cruise, Follow, ShiftOut/Pass/Return and Rejoin normal commands are
  canonical velocity-progress five-state MPCC production domains.
- Rejoin retained authority is intentionally unavailable.
- Existing user-owned change: `aichallenge/result-summary.json`.

## Invariants

- Use a clean `make dev2` start; do not manually publish initial pose/control.
- Do not change speed, wall/gap margin, solver, cadence or weights to force a
  phase observation.
- An absent phase is `NOT EXERCISED`, never a pass.
- A fresh miss may continue only through a qualified same-formulation artifact
  or explicit Emergency; legacy normal/direct ownership is forbidden.
- Stop at the earliest failed invariant and investigate upstream before coding.
- Do not commit generated output or the user-owned result summary.

## Definition of done

- ShiftOut, Pass and Return each have positive final-decision coverage, or the
  exact upstream reason preventing a phase is recorded.
- Every exercised normal Overtake decision has one matching five-state
  problem/solution/plan/execution-certificate identity.
- DynamicWait/DynamicEscape are classified as exercised or not exercised.
- Legacy normal, three-state, low-speed direct and split-authority decisions
  are zero in accepted Overtake scope.
- Callback timing, physical rejection and phase-transition evidence are
  recorded without parameter changes.

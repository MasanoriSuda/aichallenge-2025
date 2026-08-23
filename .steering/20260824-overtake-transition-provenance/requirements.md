# Requirements

## Objective

Make an Overtake canonical plan's selected lateral homotopy part of its exact
problem and lifecycle identity, and make runtime evidence distinguish an
incoming worker plan from the previously stored plan.

## Observed failure

`output/20260824-045351` eliminated the shared retained course-frame defect but
left transition-local `progress-rejected`, `initial-corridor-violation`, and
`stage-corridor-violation` outcomes. The current evaluator mutates one result
first with the incoming plan and then with the stored plan, so the final log
cannot identify which artifact failed. In addition, an early ShiftOut side
replan changes `pass_side_sign` without changing Mission generation, while the
canonical problem/lifecycle identity contains no side.

## Scope

- Canonical MPCC problem identity for Track, Cruise, Follow and Overtake.
- Overtake async context and stored-plan lifecycle.
- Overtake incoming-versus-stored current-world telemetry.
- Deterministic regression tests for a same-phase side change.

## Constraints

- Do not change safety margins, corridor tolerances, solver weights, timeouts,
  leases, fallback policy, publisher authority, or ROS interfaces.
- Do not accept any plan that fails the existing current-world proof.
- Do not promote Overtake canonical authority in this Slice.
- Preserve the user's `aichallenge/result-summary.json` modification.

## Acceptance

- ShiftOut/Pass/Return context is incomplete unless its side is exactly -1 or
  +1; non-overtake normal intents use side 0.
- Changing side while intent, target and Mission generation stay unchanged
  advances the async context epoch.
- Context invalidation removes the stored plan from the old semantic family.
- Retained proof rejects a cross-side artifact before wall/corridor evaluation.
- Logs separately report incoming and stored outcomes and selected source.
- Existing tests and build pass; `make dev2` shows no old-side plan selected
  after a side transition.

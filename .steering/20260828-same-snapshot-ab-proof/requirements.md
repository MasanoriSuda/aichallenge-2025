# IM-3 same-snapshot A/B proof requirements

## Baseline

- Branch: `develop_july`
- Rollback commit: `99dfb746`
- Production command authority, configuration and solver policy remain frozen.

## Objective

Evaluate the persistent source candidate (A) and independently rebuilt
stateless candidates (B-left/B-right) from one immutable Interaction Snapshot
through:

1. independent instances of the unchanged seven-state SQP;
2. the same exact nonlinear execution-trajectory conversion;
3. exact swept static-wall proof; and
4. exact timed dynamic-opponent proof.

The comparison is offline and cannot publish or promote a candidate.

## Failure discovered before implementation

The IM-1 snapshot preserved the control-prefix poses and an already expanded
QP wall footprint, but did not preserve:

- elapsed time for every prefix pose; or
- the raw physical ego footprint consumed before hard wall clearance is
  applied.

Those fields are required for an exact timed opponent proof and for rebuilding
the physical wall proof without double-applying clearance.  Guessing either
would violate the same-world comparison invariant.  IM-3 therefore first
closes this replay-evidence contract.

## Scope

- Add prefix elapsed time and raw physical footprint to `ReplayWorld`.
- Serialize, load, validate and fingerprint both fields.
- Bind them from the existing canonical current-control path and physical wall
  snapshot without changing live decisions.
- Add a pure same-snapshot A/B runner and CLI.
- Produce a `ManeuverBundle` only when solver, exact trajectory, wall proof,
  opponent proof and terminal-successor viability all pass.
- Report typed rejection stage and comparable metrics for every arm.

## Non-scope

- No production controller hook, mailbox, plan-store or publisher.
- No Mission resume rule, lease, grace, timeout, fallback or retry.
- No solver tolerance, weight, horizon, clearance or configuration change.
- No C lattice/polynomial arm and no D nonlinear arm.
- No architecture promotion based on synthetic unit evidence.

## Definition of done

- Mutation or omission of prefix timing/raw footprint invalidates the replay
  seal.
- A and both B sides consume the same source interaction fingerprint.
- Each arm owns a fresh solver context; no warm start crosses arms.
- A bundle cannot exist after solver, exact trajectory, wall, dynamic or
  successor rejection.
- Focused tests, all package tests and `make autoware-build` pass.
- The controller target remains unlinked from the comparison library.

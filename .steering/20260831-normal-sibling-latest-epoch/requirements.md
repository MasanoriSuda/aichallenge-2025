# Requirements: latest-epoch normal sibling coverage

## Objective

Prevent a stale in-flight normal-avoidance sibling solve from suppressing the
opposite homotopy for every newer Cruise/Follow world epoch.

## Frozen evidence

- Baseline: `fb15c4b0`
- Run: `output/20260831-033922/d1`
- First authority loss: control decision `1698`
- Frozen Follow snapshot:
  `mpcc_architecture_snapshots/000000001077-72f97ec7d9b7a11a-follow-side-negative-wall-refinement-coupled-solve-rejected/snapshot.yaml`

The visible `SafetyBrake` at decision `1705` is downstream. At decision
`1698`, while the front gap was still 3.67 m, Cruise retention lost its
terminal contingency and the proposed Follow intent had no current-world
authority. Emergency Stop then published before the V2X behavior entered its
physical emergency state.

## Architecture classification

The frozen snapshot was replayed with the repository architecture comparator.

- A, selected live/persistent branch with the seven-state SQP: rejected.
- B on the selected side with the same seven-state SQP: rejected.
- B on the opposite stateless side with the same seven-state SQP: accepted.

Classification: **live scheduling/lifecycle defect**. The opposite physical
solution exists in the same immutable world, but live execution did not
evaluate it. The bounded sibling executor reported repeated `busy` rejection,
because one older running epoch prevented the current epoch from entering the
executor.

## Invariants

- Primary certification and publication remain nonblocking.
- The sibling worker keeps at most one running and one latest pending epoch.
- A newer pending sibling replaces only an older pending sibling; it never
  interrupts a solver call.
- A sibling may merge into the branch bank only under its exact immutable
  source identity.
- A stale completed sibling cannot replace a newer candidate.
- No production authority, solver setting, timeout, lease, grace, fallback,
  velocity policy, wall clearance, or safety margin changes.

## Definition of done

- Busy rejection no longer discards all newer sibling epochs.
- Telemetry reports sibling submitted/replaced/started/completed/exception
  counts.
- Source-contract tests require latest-only sibling scheduling and prohibit a
  wait in the primary producer.
- Build and package tests pass.
- Dynamic acceptance observes current-epoch sibling coverage without source
  rollback or authority regression.

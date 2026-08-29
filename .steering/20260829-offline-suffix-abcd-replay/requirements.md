# Requirements

## Objective

Classify retained-suffix failures from existing immutable v2 architecture
snapshots without adding online solver work or changing production authority.

## Scope

Extend the existing `mpcc_prepared_suffix_replay` diagnostic to report four
explicit suffix formulations from the same recorded preparation and time
probe:

- A: old final QP with latest x0 only;
- B: common-clock time-aligned retained suffix;
- C: B semantics with a reachable nonlinear candidate and one SQP;
- D: the same C problem with a bounded offline multi-SQP audit.

These suffix labels are narrower than the full persistent-Mission versus
stateless-ManeuverBundle architecture comparison. The tool must state that
boundary and must not claim a complete Mission-lifecycle classification.

## Prohibited changes

- no controller, Store, mailbox, worker or publisher connection;
- no online solve or capture path;
- no parameter, tolerance, clearance or OSQP setting change;
- no synthetic success substitution for a failed physical proof;
- no editing or committing generated architecture snapshots.

## Acceptance

- one invocation reports A/B/C/D numerical and exact-physical outcomes;
- a failed candidate remains a successful diagnostic invocation;
- malformed/incomplete snapshots remain hard CLI errors;
- representative ShiftOut, Pass, Follow and Cruise v2 snapshots are replayed;
- results identify whether the next investigation belongs to clock/lifecycle,
  reachable candidate generation, single-SQP approximation or frozen
  physical infeasibility.

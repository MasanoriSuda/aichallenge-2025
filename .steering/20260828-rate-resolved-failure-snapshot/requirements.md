# Requirements: Rate-resolved failure snapshot

## Objective

Capture the first representative seven-state Overtake failure in a form that
can replay the exact numerical problem.  This is evidence infrastructure for
the architecture A--D comparison; it must not change production authority.

## Frozen baseline

- Source baseline: `f1734886` (production controller identical to
  `b6da7ebb`).
- Dynamic baseline: `output/20260828-001351`.
- Existing generated result files remain user-owned and are not committed.

## Failure family selected from the baseline

The first accepted ShiftOut later produced all of the following before any
Pass transition:

- exact physical trajectory rejected for lateral-bound mismatch;
- dynamic-obstacle refined QP maximum-iteration rejection;
- SafetyBrake pause followed by an opposite-side Mission generation;
- continued ShiftOut without Pass/Return.

The recorder must distinguish the numerical pipeline stage and retain the
problem that actually failed.  A log-only description is insufficient.

## Constraints

- No authority, fallback, timeout, lease, grace, clearance, solver tolerance,
  weight or horizon change.
- At most one snapshot per `(intent, pipeline stage, failure outcome)` per
  process, so evidence I/O cannot become a control-loop workload.
- Snapshot I/O occurs only on an already rejected Overtake result.
- Every snapshot records immutable semantic identity, assembled QP, warm
  start, solver outcome and static-world identity.
- The exact occupancy grid is stored as a binary payload rather than expanded
  into logs.
- Files are written below the current run directory and remain generated
  artifacts.

## Definition of done

- Failure-first serializer and exact-QP replay tests pass.
- Package build and complete package test pass.
- `make dev2` emits at least one validated failure snapshot.
- Replaying the payload gives a typed result without production changes.
- The snapshot is registered in the architecture experiment registry.

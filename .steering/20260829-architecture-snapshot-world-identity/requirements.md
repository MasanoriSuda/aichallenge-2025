# Requirements

## Objective

Preserve the first failure of each canonical `(intent, pipeline stage,
outcome)` family from the current process without silently reusing an artifact
from an older run whose candidate sequence happens to be the same.

## Repaired invariant

The pathname of a replay-ready v2 architecture snapshot identifies the
immutable interaction fingerprint stored in that snapshot. A pre-existing
artifact may suppress a write only when it represents that same interaction.

## Scope

- Architecture-audit artifact naming and deterministic tests.
- No normal authority, solver, candidate, timeout, tolerance, clearance or
  fallback change.

## Acceptance

- Replay-ready snapshots include their immutable interaction fingerprint in
  the directory name.
- The existing one-per-failure-family-per-process bound remains in force.
- A bounded run records the current ShiftOut failure rather than pointing at
  an older run's sequence-colliding artifact.

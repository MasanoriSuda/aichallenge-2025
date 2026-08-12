# Design

## Current problem

`can_preserve_committed_pass_behavior()` and
`can_preserve_committed_shiftout_behavior()` each repeat target continuity and
hard-fault checks. Pass also combines Mission readiness, latch/handoff
authority, current-overlap grace, and hard faults in one boolean expression.
That makes it risky to add ContactContinuation as a narrowly bounded geometry
exception without accidentally bypassing target, map, emergency, or solver
guards.

## Refactoring

Introduce two pure resolution steps:

1. `resolve_committed_behavior_ownership_guards()`
   - Resolves target identity availability.
   - Resolves the shared hard-fault set.
   - Is used by both ShiftOut and Pass ownership.
2. `resolve_committed_pass_geometry_ownership()`
   - Resolves ordinary latch authority.
   - Resolves body-clear handoff authority.
   - Resolves the existing unconfirmed-overlap grace.
   - Resolves whether current geometry can own Pass behavior.

The existing boolean entry points compose these resolutions with their
phase-specific prerequisites, so callers and runtime behavior remain
unchanged.

## Follow-up boundary

The next performance task may add recoverable ContactContinuation as one
additional Pass geometry ownership source. It must still pass the common guard
resolution; this refactor intentionally does not add that behavior yet.


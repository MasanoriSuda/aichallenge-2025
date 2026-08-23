# Hold/Stop intent provenance requirements

## Purpose

Before building Hold/Stop MPCC bounds, make the canonical intent preserve the
actual execution semantics already selected by the authority orchestrator.

## Root invariant

`DynamicWait` is not a longitudinal Hold intent. It describes lateral path
ownership while a Mission is being reconsidered. Both a rolling forward prefix
and a held lateral Mission path retain the original committed ShiftOut/Pass
intent. SafetyBrake remains canonical Stop and an explicit supervisor override.

## Scope

- Pure AuthorityRequest/AuthorityResolution to ControlIntent resolution.
- DynamicWait origin-phase provenance in the authority request and trace.
- Failure-first tests for lateral-hold and rolling ShiftOut/Pass continuation,
  missing lateral authority, unsupported origin, SafetyBrake and Cruise/Track.
- Replace the private controller switch with the pure resolver.

## Non-scope

- Hold or Stop QP bounds or invention of a longitudinal Hold producer.
- Publisher/authority promotion.
- SafetyBrake acceleration changes.
- Parameter tuning, fallback, timeout or lease changes.

## Acceptance

- A rolling DynamicWait with pass floor can never be fingerprinted as Hold.
- A DynamicWait without lateral authority or committed origin fails closed as
  Unknown.
- Every authority trace names the canonical intent and typed resolution reason.
- No command, path, velocity or steering behavior changes.

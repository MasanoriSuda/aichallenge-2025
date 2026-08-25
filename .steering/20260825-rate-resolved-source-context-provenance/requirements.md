# Requirements

## Purpose

Prepare the rate-resolved six-state Track/Cruise artifact for a later atomic
production promotion without fabricating a publisher identity from the current
cycle or from the old five-state command contract.

## Root cause

The asynchronous solver snapshot is created from a complete sealed
`MpccProblemContext`, but `ExecutionArtifact::Identity` reduces it to a
fingerprint plus a few copied fields.  A retained command can therefore prove
that a fingerprint existed, but the final control-decision contract cannot
recover and independently validate the source schemas, horizon, generations,
target and side which produced that fingerprint.

Using the current cycle's context would be incorrect: retained execution is
expected to keep the original solver problem identity while receiving a new
current-world execution certificate.

## Required invariants

- One complete sealed six-state source context crosses solver, artifact,
  physical proof, retained proof and command-candidate boundaries unchanged.
- Identity validation fails if the context is incomplete, non-six-state, or if
  any cached summary disagrees with it.
- Do not add a timeout, fallback, tolerance, flag or parameter change.
- Do not promote production authority in this Slice.
- Do not modify or stage `aichallenge/result-summary.json`.

## Definition of Done

- Partial duplicated identity fields are replaced by the complete source
  context.
- Tests prove context preservation and mutation rejection across the complete
  evidence chain.
- Package tests and `make autoware-build` pass.
- A short `make dev2` run shows only complete six-state candidates, no identity
  rejection and no callback overrun attributable to this refactor.

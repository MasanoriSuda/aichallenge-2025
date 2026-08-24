# Requirements

## Purpose

Give every rate-resolved six-state Track/Cruise solve an explicit canonical
formulation identity before production authority is considered.

## Root cause

The six-state request is currently derived beside the five-state problem, but
its immutable source context is sealed as `VelocityProgress5State`. The command
candidate separately labels itself as six-state without proving that label from
the solver artifact. A selected six-state command would therefore not share one
formulation identity with its problem fingerprint and physical certificate.

## Scope

- Add the rate-resolved six-state formulation to the canonical execution
  contract.
- Seal a separate six-state problem context for the six-state request.
- Carry the formulation in solver, artifact, physical, retained, and command
  identity.
- Remove the duplicate command-only formulation enum.
- Keep the candidate shadow-only.

## Non-scope

- No production publisher connection.
- No five-state Track/Cruise owner deletion in this Slice.
- No solver, weight, clearance, timeout, lease, or fallback change.
- No Follow, Overtake, Rejoin, Emergency, or Recovery authority change.

## Definition of Done

- A failure-first source-contract test rejects the old five-state context.
- An artifact with unresolved or five-state formulation fails closed.
- The command candidate derives its formulation from the certified artifact.
- `make autoware-build`, package tests, and `make dev2` pass.
- Runtime telemetry shows six-state formulation and shadow-only authority.

## Rollback

- Baseline commit: `3f63b8a fix(mpcc): unify retained control time origin`

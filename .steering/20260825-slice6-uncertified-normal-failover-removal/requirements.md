# Requirements

## Objective

Delete the remaining uncertified normal-command producers used after a canonical MPCC failure and
make every such cycle an explicit Emergency supervisor decision.

## Earliest violated invariant

A normal command must come from one solved, finite, constraint-valid and physically certified MPCC
artifact.  The current solver crawl, Dynamic Escape bounded continuation and qualification hold
construct speed/steering commands without a complete canonical solution identity.  The final trace
therefore labels them `LegacyNormalBypass`.

## Scope

- Delete simulation solver crawl and Dynamic Escape solver-failure continuation.
- Delete the one-cycle Dynamic Escape qualification hold that publishes the previous command after
  invalidating its canonical identity.
- Remove their runtime config, source enums, helpers, telemetry fields and dedicated tests.
- Remove `LegacyNormalBypass` from the final execution contract.
- Classify solver fallback and executed-solution wall hold as explicit Emergency overrides.
- Preserve canonical fresh/retained authority, Emergency stop and Stuck/gear/reverse Recovery.

## Non-scope

- No solver, weight, wall, clearance, timeout, rate or horizon tuning.
- No new fallback or retry path.
- No change to the canonical same-formulation retained-plan stores.
- No change to reverse/stuck Recovery.

## Definition of done

- Failure-first source contract rejects every retired normal bypass representation.
- An MPCC failure cannot publish a positive-speed hand-written crawl/continuation command.
- Final decision classes are canonical normal, Emergency, Recovery or disabled only.
- Focused tests, package tests and the Docker build pass.

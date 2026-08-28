# Requirements: recorded prepared-suffix replay

## Objective

Measure the time-aligned prepared-QP feedback formulation on frozen 20-stage
production failures before adding any live shadow or production authority.

## Scope

- Load the already-recorded semantic source, final `assembly_request`, exact QP
  and current-problem warm-start provenance from a v2 architecture snapshot.
- Reconstruct a deterministic later observation from the recorded primal only
  for offline timing and formulation classification.
- Compare the reduced prepared-suffix solve with a full semantic pipeline
  evaluation from the same immutable snapshot.
- Keep all replay APIs observation-only.

## Non-goals

- no production Store, mailbox or publisher connection
- no solver, clearance, lease, timeout or fallback change
- no claim that interpolated recorded state is a real later observation
- no v1 snapshot migration by guessed obstacle identity

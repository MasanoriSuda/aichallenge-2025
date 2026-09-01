# Requirements

## Objective

Collect one run-disjoint four-domain speed-committed-teacher rollout for
validation-only peer interaction evidence.

## Constraints

- Use the unchanged official-derived deterministic `e2e-final` world and the
  already-qualified teacher in all four domains.
- Keep production artifacts and authority defaults unchanged.
- Require Finish 6/6, zero penalties, zero stalls and exact runtime provenance
  in every domain.
- This is an independent execution replicate, not a randomized world; record
  that limitation explicitly.
- Do not train on or relabel the run in this Slice.

## Definition of Done

- All four bags finalize and result artifacts exist.
- Strict competition and motion gates pass for every domain.
- The rollout is admitted or rejected as validation-only evidence.

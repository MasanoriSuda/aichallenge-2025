# Requirements

## Objective

Turn the admitted four-peer `speed_committed_teacher` run into an auditable
training source and determine whether it adds the interaction states missing
from the existing one-ego/two-NPC corpus.

## Constraints

- Keep production v11, recurrent authority and all launch defaults frozen.
- Treat all four domains as one correlated training run; do not split domains
  from the same world across train and validation.
- Require the strict run-level certificate for every extracted domain.
- Preserve every scan, causal latest-preceding wheel speed and teacher temporal
  state; active-only and novelty-only filtering are prohibited.
- Keep seed 2033 and independent production-normal data as validation evidence.
- Do not promote a model from extraction or an offline metric alone.

## Definition of Done

- d1 through d4 are extracted with distinct sequence identities and matching
  executed-success certificates.
- No future, missing, stale or non-finite speed sample is admitted.
- Label and supervisor-state coverage are compared with the existing seed-2034
  training source.
- The next training experiment is selected from measured coverage rather than
  assumed from the successful race result.

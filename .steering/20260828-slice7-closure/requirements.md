# Requirements

## Objective

Close the evidence-driven Slice 7 tuning campaign without accepting a
parameter change that weakens complete Overtake execution.  Record the frozen
production baseline, all tested parameter families, their rollback state, and
the remaining non-tuning defects.

## Constraints

- Preserve canonical seven-state normal authority.
- Preserve `mpc.N=20` in local and cloud configuration.
- Preserve 40 Hz production solve submission and current-world validation.
- Do not change weights, clearances, solver tolerances, Mission lifecycle,
  fallback, timeout, lease, or grace period while closing the campaign.
- Do not classify a faster isolated run as an accepted tune when an
  independent run loses `Pass -> Return -> Idle` or adds Recovery/wall faults.

## Definition of Done

- Every Slice 7 experiment is present in the experiment registry with an
  explicit rejection and revisit condition.
- Production source and configuration contain none of the rejected candidate
  implementations or values.
- The accepted structural baseline and remaining quality defects are stated
  separately.
- JSON and documentation consistency checks pass.

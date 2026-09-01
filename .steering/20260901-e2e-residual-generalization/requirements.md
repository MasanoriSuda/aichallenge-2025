# Residual generalization requirements

## Objective

Determine whether the rejected steering residual is blocked by insufficient
run-level coverage rather than another runtime threshold or a larger temporal
network.

## Invariants

- Keep the admitted TinyLidarNet checkpoint and all production defaults frozen.
- Teacher runs are diagnostic data sources, never production authority.
- Admit a source bag only after Finish and post-start stall checks pass.
- Keep scenario seeds disjoint between train and validation.
- Do not select a checkpoint using a failure tail that is also treated as its
  validation evidence.
- Do not run a residual closed loop until material, anchor and independent
  normal offline gates all pass.

## Definition of Done

1. Obtain successful precontact-teacher NPC runs for two different seeds.
2. Add one seed to train and the other to validation without sequence overlap.
3. Re-run the frozen scan-delta architecture and offline admission.
4. Classify failure as data coverage, model/feature limitation, or a viable
   production candidate without changing runtime thresholds.

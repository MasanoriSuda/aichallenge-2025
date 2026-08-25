# Design

## Current flow

```text
tactical worker -> selected six-state side
                -> causal six-state execution worker
update_v2x / five-state Gate A -> FSM mutation
normal get_control -> consume six-state result too late
```

The five-state Mission and canonical plan therefore remain the only artifacts
available when the FSM decides whether to enter ShiftOut or Pass.

## Proposed observation flow

```text
tactical worker -> selected side + Mission geometry
                -> causal six-state execution worker
                -> result completion
next live cycle:
  evaluate_v2x_behavior
  -> consume result
  -> tactical identity join
  -> current-world revalidation
  -> typed Gate-A shadow proposal
  -> update_overtake_line (proposal deliberately ignored)
```

`RateResolvedPreentryGateAShadowProposal` owns all evidence which must later be
admitted atomically:

- selected Mission geometry;
- six-state `CertifiedPlan`;
- target and side;
- prospective Mission generation;
- tactical source sequence and async context epoch.

No five-state execution certificate is copied into the Mission. The proposal
remains observation-only and cannot mutate the normal certified-plan store.

## Promotion boundary after dynamic proof

A later Slice may replace the fresh-entry section of `update_overtake_line()`
with this proposal. That authority Slice must, in one change:

1. validate the proposal at Gate A;
2. freeze only the Mission geometry needed by the supervisor;
3. transition to ShiftOut or Pass;
4. let the existing six-state atomic intent-transition admission certify the
   actual post-transition problem;
5. delete the five-state branch selection, five-state canonical pre-entry plan
   and their entry cache/admission path.

The current Slice intentionally stops before step 1 gains authority.

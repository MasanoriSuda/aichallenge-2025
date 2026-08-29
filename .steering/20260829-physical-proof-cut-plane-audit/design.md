# Design

## Audit F

F is a bounded constraint-generation loop:

```text
reachable C problem
  -> solve
  -> immutable artifact
  -> exact physical proof
       accepted: finish
       invalid lateral sample j:
         locate (transition, substep, ratio)
         add its nonlinear lateral tangent
         relinearize and solve again
       anything else: reject
```

The cut identity is `(transition_stage, substep_index, substep_count)`. A cut
at the transition endpoint is rejected as redundant because endpoint boxes
already own that contract. Duplicate cuts are rejected rather than silently
consuming another iteration.

Each cut uses the same canonical midpoint-integrated transition and 10 ms
maximum substep contract as the physical adapter. Bounds use the same linear
interpolation between the frozen stage endpoint wall intervals.

## Interpretation

- F accepts with few cuts: dense E was primarily an inefficient/poorly
  conditioned representation; proof-guided candidate generation is viable;
- F solves repeatedly but exact proof moves to new samples: the wall tube
  needs multiple active cuts or a nonlinear formulation;
- F reaches solver rejection after a cut: the local linearized suffix is not
  accepted by the live solver under unchanged settings;
- the first rejection cannot be mapped: model/certificate provenance defect;
- Cruise remains solve-rejected without a cut: separate non-wall cause.

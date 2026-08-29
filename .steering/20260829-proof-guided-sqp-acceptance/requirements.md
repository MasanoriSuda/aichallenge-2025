# Requirements: proof-guided SQP acceptance

## Objective

Prevent a later numerical SQP iterate from replacing an already-certified
trajectory.  Compare the baseline and each bounded dynamic-SQP depth through
the unchanged exact wall, dynamic-obstacle, and terminal-successor proof chain,
and retain the first certified candidate.

## Constraints

- Observation-only; production authority remains unchanged.
- Same immutable world/problem fingerprint, candidate, homotopy, bounds,
  tolerances, objective, and physical proofs at every depth.
- No new timeout, lease, fallback, clearance, solver tolerance, or parameter
  tuning.
- Maximum depth remains the existing three-iteration audit budget.
- A later iterate may not overwrite a certified depth-0/earlier artifact.

## Definition of done

- Architecture comparison reports depth 0 through 3 independently.
- Depth 0 acceptance short-circuits all later numerical work.
- A rejected depth cannot create a ManeuverBundle.
- Frozen failures demonstrate whether proof-guided iteration recovers more
  candidates without regressing baseline-certified candidates.
- Promotion, if any, must replace an old production source in the same Slice;
  this audit Slice itself has no authority path.

# Requirements

## Objective

Close the canonical seven-state MPCC model/proof gap which removes fresh and
retained normal authority during ShiftOut after wall and dynamic-obstacle
refinement.

## Evidence boundary

- Baseline: `ce9a21d`.
- Run: `output/20260827-185648`, Domain 1, episode 1.
- Last continuously certified ShiftOut source: before decision 1844.
- First abnormal authority decision: 1844.

At decision 1844 the final QP had solved its wall/dynamic rows, but exact
nonlinear replay rejected the current stage as `invalid-lateral-bounds`.
Fresh artifacts had already been withheld for the same reason, the previous
source aged out, retained current-stage proof failed, and explicit Emergency
became the only authority.

## Constraints

- Do not change wall margin, solver tolerance, weights, horizon, lease or
  timeout.
- Do not weaken exact nonlinear replay or current-stage physical proof.
- Do not add a legacy/fallback normal command source.
- Preserve the final wall and dynamic-obstacle rows while correcting the
  temporal model.

## Definition of Done

- The final physically refined primal receives production-equivalent exact
  replay before artifact publication.
- A current-problem-owned nonlinear relinearization and solve is requested
  only after that exact replay rejects the refined primal.
- The correction preserves the same costs, boxes, wall rows and dynamic rows.
- A failure at correction assembly/solve remains fail-closed.
- Failure-first tests demonstrate that physical refinements are always proved,
  and that a rejected proof can only be corrected with same-problem-owned
  state/input provenance.
- Full build/tests pass.
- `make dev2` observes ShiftOut, Pass, Return and Idle without the current-
  stage model/proof authority hole.

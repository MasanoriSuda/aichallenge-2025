# Requirements

## Objective

Close the submission-evidence gaps found during the review of HEAD
`16b8058b`, without changing the qualified production controller, model
artifacts, launch defaults, or driving parameters.

## Required guarantees

- A video competition report must bind the raw and spatial checkpoint
  identities used by the run.
- A video competition report must bind spatial and recurrent authority state;
  disabling the production spatial adapter or enabling recurrent authority
  must be detectable.
- `multi-vehicle-candidate` requires a complete mixed-peer competition Gate,
  not motion admission alone.
- Slides and checklists must describe the implemented steering saturation and
  the actual evaluation population precisely.
- Existing generated result files and user-owned worktree changes are outside
  this slice.

## Non-goals

- No model retraining or checkpoint replacement.
- No controller, launch default, speed, acceleration, braking, or steering
  parameter change.
- No promotion of recurrent authority or failed peer teachers.

## Definition of Done

- The competition analyzer rejects wrong spatial identity and authority state.
- The readiness auditor cannot promote a peer run without a passing
  competition report.
- Unit tests cover the new fail-closed behavior.
- Submission slides and video instructions match the executable checks.
- The complete TinyLidarNet test suite passes.

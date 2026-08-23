# Requirements

## Objective

Eliminate the semantic split in which an Overtake branch is admitted with a
wall certificate derived from a lossy legacy projection of a five-state MPCC
solution, then the live execution path is rejected by a different physical wall
interpretation after ShiftOut has begun.

## Scope

- Trace the selected five-state solution through branch validation, Mission
  entry, live solve validation and published control.
- Preserve solved lateral, lag, heading and progress provenance in the physical
  execution certificate.
- Make entry admission and live execution use the same trajectory semantics.
- Add failure-first tests which distinguish the exact five-state pose sequence
  from its lateral-only legacy projection.
- Improve rejection evidence only where it identifies the same contract; do not
  introduce a second wall authority.

## Constraints

- Do not tune YAML parameters or wall clearances.
- Do not add fallback, retry, timeout, lease, tolerance or feature flags.
- Do not weaken physical footprint or swept-path wall validation.
- Do not promote a new authority in this Slice.
- Preserve the user's unrelated `aichallenge/result-summary.json` change.

## Acceptance criteria

- Overtake branch certification does not call
  `convert_extended_solution_to_legacy` before physical wall proof.
- A physical execution certificate carries the exact five-state stage pose and
  progress data required to reproduce the proof.
- Entry revalidation either validates that exact artifact in the current course
  frame or rejects it before `Idle -> ShiftOut`.
- Live five-state solutions are physically checked before legacy command
  adaptation.
- Focused tests, the package suite and build pass.
- In `make dev2`, no Mission may be admitted from a lateral-only physical
  certificate; wall rejection must be attributable to a fully identified exact
  trajectory artifact.

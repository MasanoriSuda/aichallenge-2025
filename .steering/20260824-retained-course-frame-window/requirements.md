# Requirements

## Objective

Eliminate the structural current-world proof failure where a retained canonical
MPCC plan is continuity-valid but its expected current state lies slightly
behind the newly measured progress and therefore outside a forward-only course
frame window.

## Scope

- Track/Cruise, Follow, and Overtake retained canonical current-world proofs.
- Course-frame provenance construction only.
- Failure classification and traceability for the remaining Overtake corridor
  rejections.

## Constraints

- Do not change MPCC weights, wall margins, target clearances, timeouts, leases,
  retries, fallback policy, publisher authority, or ROS interfaces.
- Do not weaken progress continuity, wall, corridor, or obstacle proofs.
- Do not promote Overtake canonical authority in this Slice.
- Preserve the user's existing `aichallenge/result-summary.json` modification.

## Acceptance

- The course-frame window covers both the current measured origin and every
  retained state that current-world proof reconstructs.
- A regression test covers the measured-ahead-of-retained case.
- Existing out-of-window and invalid-provenance cases still fail closed.
- Build and package tests pass.
- In `make dev2`, `course-frame-unavailable` caused by the lower boundary is
  eliminated without increasing wall/corridor bypasses.

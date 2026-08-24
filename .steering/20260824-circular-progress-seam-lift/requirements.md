# Requirements

## Objective

Make retained canonical current-world proof use one mathematically valid
circular-progress contract across the course seam. A finite course projection
at `0`, `path_length`, below `0`, or on an equivalent unwrapped lap must lift
to the branch nearest the retained plan instead of creating a normal-authority
gap.

## Failure-first evidence

Bounded run `output/20260824-224725`, Domain 1, contains two repeatable Follow
authority losses:

- decision 4202 at `wp_id=349`;
- decision 7733 at `wp_id=349` one lap later.

Both incoming and retained plans fail with
`current origin rejected: invalid-input`. Before calling the lift function the
controller has already established a finite current progress, finite positive
path length, a finite retained-plan progress, and the fixed finite continuity
tolerance. The remaining `InvalidInput` branch is the circular-only range test
which rejects `measured_progress < 0` or `measured_progress >= path_length`.
The reference-path seam legitimately produces an endpoint coordinate at the
path length; rejecting that equivalent representation removes all normal
authority until projection wraps to the first waypoint.

The later steering-continuity rejection is downstream: Emergency holds the
published steering while retained plan time advances. It must remain visible,
but is not the producer of this repeatable seam gap.

## Constraints

- Do not tune steering rate, gap, wall, solver, timeout, lease, or grace.
- Do not add a Follow-only seam exception.
- Preserve the discontinuity test after lifting to the nearest circular branch.
- Keep non-circular behavior unchanged.
- Accept only finite progress and finite positive path length.
- Do not modify or commit `aichallenge/result-summary.json`.

## Definition of Done

- One pure circular lift accepts `0`, `path_length`, negative seam values, and
  equivalent multi-lap representations when they are continuous with the
  retained reference.
- It still rejects a nearest lifted branch beyond the continuity tolerance.
- Existing non-circular and ambiguous-tolerance behavior is unchanged.
- Package build and all tests pass.
- A bounded multi-lap `make dev2` run has zero seam-origin `InvalidInput`
  Follow authority losses.

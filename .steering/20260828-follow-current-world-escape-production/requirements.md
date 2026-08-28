# Requirements

## Objective

Promote the frozen A/B finding into the existing single canonical normal MPCC
pipeline: when a Follow stay-behind solve cannot produce a certified artifact,
evaluate bounded stateless current-world side candidates and allow only a
fully certified selected artifact to enter the existing candidate store.

## Evidence

Snapshot 1755 proved that persistent Follow A failed while both stateless side
B arms passed the unchanged seven-state SQP, wall proof and exact dynamic
proof. The negative-side arm retained materially more terminal progress and
velocity.

## Constraints

- Keep Follow intent and semantic identity unchanged; candidate side is not a
  Follow intent attribute.
- Run only inside the existing latest-only normal worker. Do not add a new
  thread, publisher, store, lease, timeout or authority class.
- Try persistent Follow first. Evaluate side candidates only when it does not
  produce a certified plan.
- Candidate solves may not share cross-side warm-start state.
- Only the selected certified plan may replace the canonical candidate store.
- Current-world retained revalidation and final serialized publication join
  remain mandatory before execution.
- Do not tune solver, clearance, speed, braking or wall parameters.

## Definition of done

- Existing Overtake stateless behavior is unchanged.
- Follow candidates are bounded to one per side and rebuilt from one sealed
  current world.
- Selection is deterministic and progress-first among certified candidates.
- Static tests prove no Follow side is written into semantic identity and no
  unselected candidate enters the plan store.
- Dynamic Gate observes a certified Follow escape or yields a new classified
  blocker without restoring the old speed-envelope failure.

# Design: current-world candidate topology

## Root cause

The current-world population confused the control horizon with the finite
target encounter. It could omit the only certifiable transition when the sealed
target tube ended before the control horizon. Captured forced schedules and a
vector-invalidated reference further made population semantics unstable.

## Design

1. Build a neutral stateless seed by deleting every captured forced topology.
2. Scan the canonical target tube's contiguous valid interval.
3. Keep DirectSide and midpoint physical diagonal candidates.
4. If the encounter ends in-horizon, add a physical diagonal from the last
   nominal stay-behind stage to the first invalid target stage.
5. Otherwise preserve the late exact-disjunction used for full-horizon targets.
6. Assemble all candidates in stable local storage, then move at most three into
   the result vector.

The new member is a candidate generator only. It has no publisher, retention,
authority, fallback, retry or special proof path.

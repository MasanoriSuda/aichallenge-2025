# Design

## Root cause

The worker problem is formed at one predicted control origin and completes
after later commands have already crossed the publisher.  The candidate's
prefix and the live command stream therefore have different histories.
Selecting a suffix by candidate age compares equal times but does not establish
equal states.

## Shadow contract

For each fresh candidate, capture the atomic executed-plan ledger and derive a
common switch control time.  Sample:

1. the parent artifact at the switch time using its first-published control
   origin and first-published artifact cursor;
2. the candidate artifact at the same switch time;
3. desired steering, response steering, Frenet state, velocity and progress;
4. immutable parent/candidate/world identities.

The result is a data-only `OnTrajectoryConnectorProof`.  It is accepted only
when both cursors exist and the two predicted physical states agree within the
existing model/certificate tolerances.  No new tolerance is introduced.

## Next production architecture

If the shadow comparison proves that an explicit future switch is viable, the
successor solve will include the certified parent prefix up to that switch and
will be assembled from the parent switch state.  The publisher continues the
parent until the switch; the child may be adopted only at that exact state.

If the candidate still diverges because current feedback changes the parent
trajectory, implement an AS-RTI-style latest-state feedback correction rather
than adding another lease or grace period.

## Non-goals

- No parameter tuning.
- No origin-cursor replay.
- No stale candidate hold.
- No second normal-control authority.
- No direct full solve inside the 40 Hz callback.

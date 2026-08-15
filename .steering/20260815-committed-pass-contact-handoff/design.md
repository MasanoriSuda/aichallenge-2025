# Design

## One-shot target-bound hold

Store the Mission generation that consumed the target-bound hold budget.
`can_hold_target_bound_execution_for_replan()` rejects another arm for that
generation. A frozen Mission replacement increments the generation, naturally
making a fresh bounded hold available without a timer or parameter change.

Normal resolution by a stable fresh horizon does not consume the one-shot
budget; only actual time/distance exhaustion latches the generation.

## Pass-origin contact context

`OvertakeMissionOwnershipResolution` distinguishes ordinary paused Missions
from a tactical `FollowPrepare` whose origin was `Pass`. This context is used
only by ContactContinuation; it does not broadly restore Pass ownership.

Before `Pass -> FollowPrepare`, copy the already latched forward-completion
evidence into a dedicated DynamicMissionWait field. Pass-local timers and
latches may still reset as before. During the pause, the contact classifier
uses the Pass-origin context, saved forward-completion latch, current target
geometry/progress and the existing wall, heading, duration and velocity guards.

## Contact execution

A recoverable side contact is an accepted current geometry for
DynamicMissionWait and its forward-prefix policy. The prefix remains on the
committed side and ramps the existing 0.15 m separation bias through the wall
and lateral-acceleration horizon evaluator. It does not reuse the invalidated
target trajectory.

The physical overlap remains a hard failure when ContactContinuation rejects
it. Atomic Mission replacement still requires separated bodies and is not
broadened by this change.

## Expected logs

- At most one `target-bound execution hold started` after a budget exhaustion
  for the same generation.
- A rear-side overlap after `Pass -> FollowPrepare` logs
  `ContactContinuation entered` rather than immediate
  `rolling replan encountered runtime hard fault`.
- DynamicMissionWait publishes a current-side prefix with forward speed until
  bodies separate; then normal rear-clear/Return processing resumes.

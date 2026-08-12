# Requirements

## Purpose

Prevent a bounded, recoverable side contact during a committed Pass from
dropping Behavior ownership to Follow or SafetyBrake while OvertakeLine is
still intentionally continuing the same Mission.

## Observed failure

The latest run entered ContactContinuation during Pass, but
`can_preserve_committed_pass_behavior()` rejected confirmed body overlap. The
Behavior FSM then left Overtake even though the front-danger policy and
OvertakeLine both accepted the same contact as recoverable. This caused speed
loss, FollowPrepare, and Mission timeout.

## Required behavior

- A validated, frozen Pass may retain Behavior ownership while
  `recoverable_side_contact_active` is true.
- ContactContinuation must not outlive its existing duration/progress limits.
- Target discontinuity, position jump, course rejection, side intrusion,
  forbidden waypoint, unsuppressed emergency risk, and solver recovery remain
  ownership-release conditions.
- Non-recoverable or expired contact remains fail closed.
- No configuration or ROS interface changes.


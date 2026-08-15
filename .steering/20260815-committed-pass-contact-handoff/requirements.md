# Requirements

## Goal

Complete an already committed overtake without allowing a bounded optimizer
gap or a recoverable rear-side contact to destroy the Pass Mission.

## Evidence

Run `output/20260815-110344/d1/autoware.log` showed:

- the 1.5 s / 8.0 m target-bound execution hold exhausted and then rearmed in
  the same Mission generation;
- the target had moved behind ego (`target_s` about -1 m), but a side-body
  overlap during `Pass -> FollowPrepare` was classified as a generic runtime
  hard fault;
- the resulting Recovery was followed by a 6.25 s SafetyBrake stop.

## Required behavior

1. Target-bound execution hold exhaustion is terminal for that Mission
   generation. A replacement Mission generation may arm a new hold.
2. A `Pass`-origin DynamicMissionWait retains enough commit evidence to
   classify a newly observed rear-side contact.
3. A contact accepted by the existing bounded ContactContinuation classifier
   keeps the same-side forward prefix and applies a wall-bounded lateral
   separation bias.
4. Return starts only after physical body separation/rear-clear. Wall contact,
   emergency front risk, stale target, excessive heading/lateral velocity,
   stalled contact and solver recovery remain fail-closed.
5. ROS topics, message types, launch structure and evaluation interfaces are
   unchanged.

## Non-goals

- Increase contact duration, closing-speed or wall-clearance thresholds.
- Permit frontal impact continuation.
- Change reverse/stuck recovery.
- Tune general overtake aggressiveness.

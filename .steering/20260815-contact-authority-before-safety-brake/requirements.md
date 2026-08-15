# Requirements

## Goal

Allow a classifier-approved, Pass-origin side contact to retain Overtake
authority before the front-hazard hold converts the same target into
SafetyBrake.

## Evidence

Run `output/20260815-113358/d1/autoware.log` showed:

- `Pass -> FollowPrepare` was followed by SafetyBrake after about 20 ms;
- `ContactContinuation entered` later while the target was rear-side, but
  diagnostics still reported `pass_owner=0` and `danger_suppress=0`;
- ego speed fell from about 6.26 m/s to 0.48 m/s during the admitted contact;
- no DynamicMissionWait prefix was published with `contact=1`.

## Required behavior

1. Recoverable side contact may supply committed execution authority during a
   Pass-origin DynamicMissionWait without requiring a previously published
   DynamicMissionWait prefix.
2. Front-danger suppression applies only when the current or held hazard target
   ID matches the frozen Mission target.
3. The suppression releases the same-target hazard hold, preserves Behavior as
   Overtake and lets the existing wall-bounded contact prefix run.
4. A different front vehicle, stale/jumped target, inter-vehicle corridor,
   unclassified overlap, wall fault, forbidden waypoint and solver recovery
   remain fail-closed.
5. ROS topics, message types, launch structure, parameters and evaluation
   interfaces remain unchanged.

## Non-goals

- Extend ContactContinuation duration or geometry thresholds.
- Suppress a frontal impact or another vehicle's emergency.
- Change general overtake candidate ranking or wall-clearance parameters.
- Change stuck/reverse recovery.

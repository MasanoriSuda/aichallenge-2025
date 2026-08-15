# Design

## Problem boundary

The tactical evaluator already compares left, right, current-side hold, and
Return candidates, and the Frenet-DP path is already consumed during ShiftOut
and Pass. The discontinuity is at the execution boundary:

1. a transient candidate/horizon miss requests DynamicMissionWait;
2. OvertakeLine changes to FollowPrepare;
3. the runtime-validation lease is cleared because authority is Pass-specific;
4. execution falls back to a two-metre current-lateral prefix;
5. Behavior observes only hard center distance and enters SafetyBrake before a
   fresh lateral solution can be installed.

## Change

### Phase-neutral execution authority

Rename the core Pass-only authority to `FrenetDpExecutionAuthority`. It admits
either active ShiftOut/Pass execution or a DynamicMissionWait originating from
one of those phases. All target, side, body, prediction, wall, solver, age, and
remaining-distance checks remain centralized in the core resolver.

### Atomic last-feasible continuation

Preserve the runtime-validation timestamp only when ShiftOut/Pass enters a
DynamicMissionWait with an active DP path. Rolling refresh remains atomic: a
rejected or missing candidate does not erase the last feasible path.

During the wait, interpolate the stored distance-domain DP path onto the MPC
horizon and run the normal wall/lateral-acceleration validation. If valid, use
that sequence directly and renew the short runtime-validation lease. If it is
invalid or expired, retain the existing measured-state prefix and Recovery
fallback.

### Behavior-to-line authority bridge

Behavior is evaluated before OvertakeLine in each control cycle. It therefore
queries the same execution authority using the prior cycle's validation lease.
The result is supplied to the existing committed-corridor front-danger
suppression. Suppression still requires exact nearest-front/locked-target
identity plus body/prediction separation; a different vehicle remains an
emergency obstacle.

## Scope and remaining work

This is the next hybrid-MPCC stage, not a full nonlinear MPCC replacement. The
FSM still selects the target and owns Return/Recovery, while the continuous DP
path owns lateral execution through soft tactical replans. A later stage may
unify longitudinal input optimization and Return in the same horizon after
dynamic results demonstrate that this boundary no longer produces the failure
tail.

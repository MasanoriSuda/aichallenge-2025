# Requirements: Overtake retained current-world proof

## Objective

Remove the remaining evidence gap before canonical five-state MPCC may own
ShiftOut, Pass, and Return. A previously certified five-state plan must be
eligible for continued execution only after its remaining window is certified
against the current control pose, wall map, target identity, target observation,
and current stage-wise obstacle corridor.

## Root-cause boundary

The current Overtake path already constructs a complete canonical command after
the exact five-state primal, actuation, trajectory, lateral-row contract, and
physical wall path have been certified. The result is shadow-only and is then
discarded in favour of the legacy conversion / three-state authority path.

Promoting that fresh result immediately is not yet justified: replay still
contains solver-failure and circuit-skip cycles. Replacing the legacy fallback
with an emergency stop on every such cycle would remove split authority but
would introduce avoidable braking during an otherwise valid pass. The missing
contract is a same-formulation retained candidate revalidated against the
current world.

## In scope

- A typed current Overtake corridor observation and stable fingerprint.
- Current-world retained proof for ShiftOut, Pass, and Return.
- Exact target, Mission generation, cursor, pose-prefix, course-frame, wall,
  and dynamic-corridor validation.
- A single immutable Overtake canonical plan store.
- Shadow telemetry that separates fresh readiness from retained readiness and
  records exact rejection reasons.
- Unit, package, build, and deterministic replay evidence.

## Out of scope

- Production authority promotion.
- New fallback, timeout, lease, flag, or parameter tuning.
- Retaining a plan across target, Mission generation, or intent changes.
- Treating age alone as sufficient revalidation.
- Removing legacy Overtake execution before dynamic evidence exists.

## Acceptance criteria

1. A retained Overtake candidate cannot be built without a current target
   observation and a fingerprinted current corridor.
2. Target, intent, Mission generation, cursor, pose, course-frame, wall, and
   every remaining stage corridor must be certified.
3. A plan outside a current corridor, behind an expired cursor, or belonging to
   a different target/intent/Mission is rejected with a typed reason.
4. Fresh post-certification failure never poisons the retained plan store.
5. This Slice remains shadow-only and cannot mutate the published command.
6. All focused and package tests pass, and replay reports fresh/retained
   coverage and rejection distributions.

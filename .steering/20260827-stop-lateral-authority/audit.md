# Audit: SafetyBrake lateral authority

## Observed symptom

`output/20260827-214537/d1/autoware.log` first showed a failed
Stop-to-ShiftOut reconnect. That failure was downstream. The earlier causal
sequence was:

1. decision 3048 entered explicit Stop at 5.15 m/s;
2. Stop kept physical steering at `-0.159 rad` while braking;
3. the reference curvature changed from about `-0.098` to `-0.032 rad/m`;
4. wall clearance degraded from clear to 0.31 m and then 0.00 m;
5. decision 3106 reported physical wall contact at 0.84 m/s;
6. only after the vehicle state had been displaced did the interrupted
   ShiftOut fail steering/progress reconnect proofs.

## Root cause

The Stop boundary removed the legacy normal solver correctly, but treated
"Stop owns the complete wire command" as "Stop must freeze lateral command".
Those statements are not equivalent. Maximum longitudinal braking has a
non-zero stopping distance, so a constant lateral command has no suffix proof.

The latent normal shadow cannot be published as the repair: it was solved for
a different longitudinal trajectory and remains intentionally non-production.

## Change mapped to cause

- Added one pure `StopLateralAction` decision so moving and at-rest semantics
  cannot be inferred independently at call sites.
- Moving Stop rate-limits the last wire steering toward the existing spatial
  reference-path feedback target.
- If that target is unavailable, moving Stop rate-limits toward neutral.
- Only an at-rest Stop holds steering.
- Stop still publishes zero target speed with Emergency authority; no normal
  artifact is marked executed and the atomic successor contract is unchanged.

No timeout, lease, grace, parameter, solver fallback, wall-margin change, or
second normal controller was added.

## Static verification

- `make autoware-build`: passed, 25 packages.
- `colcon test --packages-select multi_purpose_mpc_ros`: 47/47 targets passed.
- `colcon test-result --verbose`: 1,938 tests, 0 errors, 0 failures, 0 skipped.
- Structural authority test confirms the Stop function cannot call normal
  production control or publish a normal command.

## Dynamic verification

Run: `output/20260827-221458`, bounded `make dev2`, approximately two minutes.

- Both domains started and drove.
- Domain 1 encountered a separate DynamicEscape wall-near/failsafe episode,
  reached about 0.21 m wall distance, did not report wall contact, and later
  returned to normal Cruise at race speed.
- Neither domain emitted an explicit SafetyBrake Stop during the observation
  window. Consequently the new `lateral=track-reference-path` path did not
  execute dynamically.

This run is a no-regression observation, not the dynamic acceptance of this
Slice. Dynamic acceptance remains open until a moving explicit Stop is
observed and its steering/wall sequence is checked.

## Remaining risk

1. The emergency path-feedback target reuses established fallback gains, but
   has not yet been observed under a real moving Stop after this change.
2. DynamicEscape can independently approach the wall under solver fallback;
   this is outside this Slice and must not be folded into Stop tuning.
3. The interrupted Overtake successor still needs a fresh current-world solve;
   this change prevents Stop from corrupting its starting state but does not
   guarantee that every old Mission remains reconnectable.

## Next dynamic acceptance

For the first run containing a moving SafetyBrake Stop, verify:

- the final decision contains `canonical_intent=stop` and
  `lateral=track-reference-path` or `lateral=neutralize`;
- physical Stop steering changes within `steer_rate_max` as curvature changes;
- Stop remains the only published authority until an exact normal successor;
- no clear-to-near-to-contact wall sequence occurs during braking;
- successor adoption occurs atomically, without an old Cruise command.

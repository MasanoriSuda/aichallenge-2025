# Results: Current-world A/B contract

## Repaired invariants

1. ReplayWorld identity is tied to the target-observation generation, not the
   unrelated ego decision generation.
2. Valid `-inf/+inf` semantic state and input bounds survive serialization and
   replay; NaN and incorrectly signed infinities remain rejected.
3. Stateless B rebuilds target stages from the selected current-world target,
   recorded stage times, unwrapped course-frame window and physical extents.
4. Persistent Mission target stages cannot influence the rebuilt B candidate.
5. A short horizon may name `Replan` when the encounter continues.  Every
   accepted successor also carries physical braking authority and a
   non-authoritative contingency Stop intent.

The world projection and exact dynamic proof both use the same recorded
constant-velocity observation.  No runtime clock, Mission geometry, lease or
retained artifact is used by the producer.

## Dynamic result

The previously blocked native snapshot now reaches the common solver in all
three arms.  A and negative-side B fail numerically.  Positive-side B solves,
then is correctly rejected by exact dynamic proof at `-0.007975 m` minimum
clearance.  No ManeuverBundle is created.

## Verification

- Failure-first build: failed because `Replan` and current-world rebuilding did
  not yet exist.
- `make autoware-build`: passed, 25 packages.
- Focused stateless/snapshot/comparison tests: 17/17 passed.
- Full package suite: 49/49 CTest targets, 1992 tests, zero failures.
- Native frozen comparator: reached common SQP and physical proof.
- Production authority linkage: unchanged; the stateless producer and
  comparator remain offline-only.

## Next gate

Production stays frozen.  The next Slice must explain the positive-side
QP-versus-dense-proof mismatch before C/D, tuning or authority promotion.

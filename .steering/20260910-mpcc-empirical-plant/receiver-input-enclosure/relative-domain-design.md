# Relative body/time domain: bounded comparison and proof obligations

2026-09-12. Production remainsfbc736c2/build102/tests89. This design does not
grant authority. Original receiver250ms, packet25ms, origin130ms, steering100ms,
all hard bounds and current/final guards stay unchanged.

R62 D1 firstpre894/source889/index2 is inside the complete global domain, but
that domain's future Cartesian pose population fails the current wall test.
DomainUseReason7 means WorldRejected, not OutsideDomain5. Full current reproof
passes, consumes27.128277ms wall/25.282655ms CPU, and crosses the original send
deadline. R60 D2pre489 is instead outside global Y/yaw. Both are exact sealed
inputs. Wider/symmetric and layered global boxes do not resolve both cases.

R373 compares a new independently propagated FULL body/time population with
normalized starting pose0, preserving original complete input history/programme,
original time domain, footprint and body coordinates3..7. It composes every
future body and swept rigid corner with the whole current-prefix pose using
outward interval arithmetic, then applies the original current-world checker.
R62 body/time/world pass269 samples (worker24.722304ms, world2.188635ms); R60
pass89 samples (worker9.329249ms, world0.481604ms). These are diagnostic costs,
not live bounds or full-race acceptance. Global alternatives remain rejected.

This differs from rejected old point-future reuse R193/R194: no old point future
is translated into a new population. Every member of the proposed body/time box
is independently propagated. Promotion still requires the following proof.

For a fixed original input history and native time grid, the flat settled-contact
kernel's body variables do not depend on global XY or heading. Translation adds
to position; an initial yaw rotation rotates each position increment and rigid
corner. At rest the native branch preserves pose and updates tire only. Inspect
shared value kernel and independent derivative/corner enclosure before relying
on this algebra. Pose composition is p_xy+R(p_yaw)q_xy and p_yaw+q_yaw. Corner
composition includes the raw observation heading, since corners use world axes.
All products/sums/trigonometric bounds must be outward rounded; no midpoint or
old point correlation may replace the whole prefix.

Required before integration:
- Kernel-level containment of native steps and swept corners across all body
  extremes, initial heading/translation (including large map origins), both
  acceleration signs, rest transitions and delayed steering histories.
- Distinct normalized-domain type/meaning. Do not weaken global membership.
  Require exact source/programme/profile/model/footprint identity, full current
  body/time membership, actual prior/suffix IDs and unchanged generation checks.
- Current-world proof checks every composed interval through rest; Follow keeps
  its existing independent first-window contract until separately proved.
- Preserve source and current proof horizons and every before/after clock guard.
  Add negatives for changed history, source, footprint, time, body, world and
  revocation; all invalid candidates remain unable to mint a receipt.
- Replace the superseded nonFollow global proof path in the same production
  slice if promoted; run native/package/build/replay then committed standardr63.

[Evidence](r62-relative-comparison-evidence.json). R62 also has separate
D2pre1104 scheduling pauses and D1post2306 final publish crossing its original
window. Relative geometry alone is not a claimed repair for those failures.

## Implementation and kernel verification

The shared value kernel is `mpcc_vehicle_model_kernel.hpp`; its interval/derivative
enclosure is `detail/mpcc_vehicle_enclosure.hpp`. Body force, COM velocity, yaw
rate and tire updates use body coordinates and input only. Heading rotates the
reference-point position derivative; translation does not enter those equations.
The midpoint map preserves this rotation relation, and its rest branch preserves
pose. The rigid-corner sweep rotates the interpolated pose and original offsets.
Thus independently enclosing all body/time starts at zero pose permits composing
every relative sample with every member of a fresh pose box. Interval products,
sums and trigonometric functions in the implemented transform round outward.
This statement concerns the existing discrete flat settled-contact model and
existing pose sweep, not an unverified guarantee about a different physical model.

A distinct RelativeProgrammeStartingDomainTube and unique fingerprint tag now
represent that theorem. RelativeDomainTransform construction checks all finite
body/time ranges, normalized pose and exact footprint offsets; its constructor is
private. It does not authenticate commands. Private StartingDomainEvidence binds
the numerical result to the original certificate. NonFollow replaces the former
global-pose full-programme worker proof; Follow retains its first-window proof.
The current dispatcher composes every sample/corner using the fresh observation
origin and applies the same complete world checker. Source ID, delayed input
memory, actual prior/suffix history, revocation, source/current rest and original
before/after receipt gates remain in place. Snapshot marks relative_pose_composition.

R374 diagnostic kernel composition passed2,764,800 native corner coordinates and
body endpoints across acceleration/braking, rest/reverse/launch, five observation
headings and large map translations. R377 repeats this using the production
transform, plus distinct fingerprint, strict original global predicate and
body/time/footprint/NaN negatives; all10 numeric tests pass. R375/376 were test
include/link setup failures and are preserved. R378 all157/158 exposed an old
fixture that used the global domain origin to teleport a point. R379 replaces
that rejected-pose assumption with a rejected body-rate population and preserves
the required complete current reproof; all158 C5/native tests pass.
Build103 passes all 26 packages. Test90 detects an unrelated worker initialization
race, fixed separately in 84155a70. Final build104 passes all 26 packages and test91
passes 2,582 records / 66 groups (105 source contracts), including the new snapshot
marker roundtrip. Replay380/381 passes nine actual-library source/prefix/clock
checks. New relative current proof IDs are intentionally distinct; original late
publication remains rejected. [Implementation evidence](relative-domain-evidence.json).
Standard dev2-r63 is next. Runtime scheduling pauses, final publication deadlines,
sustained positive programmes and all remaining M4-M6 acceptance remain open.

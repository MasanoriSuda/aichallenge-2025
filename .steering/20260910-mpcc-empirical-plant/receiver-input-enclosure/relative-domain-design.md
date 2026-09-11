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

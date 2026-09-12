# Fresh contact validation and observation epochs

2026-09-13 JST. Production8d9810e0/build120/package107 unchanged; run was launched
from documentation HEADc5afc239. No production candidate is promoted. FullM4–M6
remain incomplete. [Manifest and payload hashes](contact-prospective-evidence.json).

## Fresh run and fixed comparison

Force-single-r2 used the validated seven-call probe and the unchanged controller.
First moving failure is D1Ready972/source643/job967/index2, before Start or laps.
The original source/domain/history/previous actual971/index1 and current checks
reproduce exactly in R553 and R557. Both current replays reject wall at12.599999726;
the check vector is[5,0,5,0,4,6]. This is a physical proof rejection before final
send, not an observed publication deadline overrun. Entry12.239999726 is inside
the original nominal12.234999827/deadline12.259999827 window. The two unloaded
current checks took3.675/3.617ms; this establishes no live timing bound.

R550 has1810force records; R551 extracts5036public rows. All157direct applications
join exact receiver sequence/source/float32 input.543public signatures associate
the sole receiver withD1.529received commands before the final apply are overwritten.
Observed selected-source age16.783–85.513ms and apply intervals100–180.513ms include
startup/teardown. They do not replace the250ms empirical receiver profile. Original
DLL and both userJSON hashes are restored; all runtime containers are stopped.

R554 follows the pre-run protocol:42anchors every20physics ticks from first awake
Drive, stopping before the first failure. Horizons0.1/0.25/0.5/1s have42/40/40/36
samples. No fitting or coefficient change on r2. Both frozen drivers reproduce
all158original predictions exactly. At1s, R549with-wire reduces uMAE0.085013→
0.041026m/s, position0.057563→0.024681m and yaw0.015305→0.011734rad. Every arm uses
the same actual future wire/tire and corrected base_link truth. Initial private
body isolates the model boundary. The older D1 regressions and complete source/
current/Stop/closed-loop acceptance remain unmet; smaller means are not a bound.

## Earliest broken boundary

R557 checks27physics instants in the original source tube. Every actual wire is
inside its original sign-group UNION, yet every speed sample is outside. At the
source pose epoch12.104999729, physical u0.094152105 already exceeds upper0.071464076.
The raw public u0.068163015 is correct at its own12.074999730epoch,30ms earlier.
Eight physical tire samples also leave the tube; this run has a substantial held
tire epoch difference, not just the r1~2e-9rad numerical discrepancy.

Producer is `predict_observed_vehicle` in mpc_controller_cpp.cpp:55580. It selects
each latest received sample at or before the pose stamp, then holds those values
at that pose epoch. Provenance records their older stamps, but that does not make
the numerical initial body synchronized. The complete current check detects the
inconsistency and then rejects the independent remaining wall proof. Emergency
is safety handling, not a mask to remove. R1's pose/velocity were aligned and still
failed, so sensor skew is a contributor alongside contact modelling, not the
single cause of every run.

R556 separates these factors on exact r2 observations. From programme pose to
the later actual velocity epoch(+110ms), original held uerror is−0.045432m/s;
private synchronized initialization gives−0.019528. Future-contact-only with held
initialization still gives−0.024719, versus+0.001186when synchronized. Both factors
matter. Frozen contact models improve synchronized u but leave yaw/vy errors.
Future contact/geometry and synchronized private state remain diagnostic oracles.

R555 scores11140current physical steps from r1/r2 and old single/dev2 windows.
Current contact/geometry removes much native error, but oldholdout lateral stepMAE
is still0.006507m/s; current3Dforce reduces it to0.000598. Yaw-rate stepMAE reduces
0.003251→0.001431rad/s. The3Dforce oracle's low-speed longitudinal error is worse
than planar contact because subsequent PhysX constraints and integration remain.
Simply adding gravity or a residual is unsupported. No eligible inertia is invalid;
the older raw extractor's unselected-state warning remains explicitly separate.

## Public reconstruction comparison

R558 uses public samples capped at the exact per-topic source stamps selected in
each saved snapshot. It resamples a common historical epoch using bracketing
endpoints, retaining both raw stamps and interpolation fractions. No IMU linear
acceleration or forward extrapolation is used. The comparison is supplemental,
after inspecting r2, and both arms end at the same pre-failure epoch.

For r2programme, mixed-model uerror improves−0.026373→−0.001501m/s. However yaw
worsens. For r1current, the original-model uerror worsens−0.001386→−0.010919 because
the common epoch lengthens prediction from5ms to50ms. R2semantic interpolation
crosses a rest/launch transition and creates0.005673m/s initial error. Thus blindly
moving every initial state to the oldest common timestamp is not a production
repair. Pose/tire endpoints are retained but absolute localization and tire/
receiver prediction are not validated by these COM/relative-motion rollouts.

## Concrete remaining work

1. Inspect the actual received component histories at the decision boundary,
   including samples newer than the selected pose that `latest_at` currently
   discards. A bag's receipt is not the controller callback's receipt. If existing
   snapshots cannot establish this, add bounded observation to the failure
   capture; preserve source/receipt/effective epochs and reset identity. Then
   compare a current, causally reconstructed state with explicit raw support.
   Do not relabel interpolated values as original measurements or interpolate
   through unobserved rest/launch/reset transitions.
2. Account for contact-regime and lateral/3D effects with a physically explained
   model/uncertainty contract. Preserve the oldD1and both new exact-source yaw
   regressions. R548/R549 coefficients stay frozen; r2is now inspected and cannot
   be reused as untouched validation for a new fit. No arbitrary residual,
   positive minimum contact assumption, timing grace or safety-margin reduction.
3. A selected repair must have one model and immutable observation identity in
   QP/native/Stop/current/async consumers. Raw and reconstructed observations need
   distinct provenance; retire the replaced holding path in the same slice.
   Run failing causal replay, negative clock/receipt/rest/context cases, source
   checks, full build121/package108 only after a production change, and fresh
   committed dynamic validation with original constraints. Current runtime
   wall/clock failures must actually pass; no point-oracle result grants authority.
4. Complete remaining M4intents/Mission/sibling/Store, real Stop/rest/restart,
   Recovery/Rejoin/Boost/async and r75Start/update_v_max. Then same-final-HEAD
   single/dev2three repeats, dev3/dev4six laps, gate1–3 and the same tar/image/eval.
   Six laps are the local criterion;2026equivalence remainsTBD. Existing user
   authorization covers this work and necessary local commits; no routine
   confirmation or push is introduced.

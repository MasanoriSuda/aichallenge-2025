# Peer epochs and observation results

Baseline 36c2a01a. Raw V2X inputs from both Domains are preserved. Actual
production motion-filter kernel reproduces both captured D2 velocities exactly.
For 928, source 9.749999782 / array 9.774104204 / receipt 1788926322.346016,
raw position (89631.9453125,43132.44921875), projection to now is 39.999999 ms.
For 929, source 9.79999978 / array 9.824104561 / receipt 1788926322.3925483,
position (89631.875,43132.50390625), projection is 20 ms. Both projected XY
errors against the captured Request are exactly zero. The tracker correctly
uses per-vehicle source time; array/receipt relabelling is not supported here.

Filtered velocity changes (-1.1513923493164182,1.0248150202914346) to
(-1.2405925467431673,1.048942278501929); raw finite differences are
(-1.2500000250000292,1.0937500218750256) and
(-1.406250056249987,1.0937500437499899). Native acceleration estimates are
(-0.7949513736934113,0.4471106442269641) and
(-1.0422145352438394,0.45596927904814527). These are estimator outputs;
filter lag, quantization, real motion and model approximation are not by
themselves proved producer bugs. No motion/gain/uncertainty change selected.

The recorder drops valid physical/artifact evidence whenever solver source is
absent. A native test supplies a real accepted straight-line physical proof,
complete immutable artifact and no solver snapshot, just as the derived Stop
path permits. Baseline testing used a passive optional plan pointer before serialization/live
capture changed. The repair now retains the actual immutable plan in the existing
observation job and writes the separate complete physical evidence on the worker.

Native baseline r1 failed to link the newly used physical/certified libraries;
the dependency omission is preserved and corrected in the helper and CMake.
Baseline r2 runs all 18 cases: 17 pass, the new real-physical-proof case fails
because complete publication evidence is absent. Serializer repair r1 had an
incorrect diagnostic stringifier name; this compile failure is preserved and
the existing typed physical-wall reason formatter is used. After-r2 passes18/18 in242ms; final after-r3 adds invalid-publication isolation
and passes18/18 in250ms. A diagnostic copy of this native fixture preserves
its generated files before cleanup; the production test is unchanged. The
source-free decoder reads its complete recorded artifact/physical snapshot/proof,
validates context/pose/frame/grid identities, and re-evaluates the wall proof:
all diagnostic fields exactly match (including NaNs); zero solver invocations.
The first round-trip launch had an incorrect filename glob and never executed
the decoder; corrected launch passes. All failed attempts remain in evidence.
Normal `make autoware-build` passes26packages in4min17s. Only existing
ament header-install advisory and setuptools deprecation stderr were emitted.
`colcon test --packages-select multi_purpose_mpc_ros` plus package-local
`colcon test-result --verbose`:60CTestgroups /2374records,0errors/failures/skips,
24.82s. Host104single-authority source-contract cases pass. No host pre-commit
is installed; Python AST, local doc links and `git diff --check` are used.

## New bounded dev2 observation

`output/20260909-peer-epochs-dev2-r1`, baseline36c2a01a with sealed observation
patch. First moving D1Emergency931 at1.3129826481757174m/s, observed at wall
1788929233.142931746. No finish/details; coupled acceptance rejected. This
observation-only slice does not repeat the preceding baseline's single six laps.
All containers stopped and both protected user JSON restored to original hashes.

First normal terminal rejection928 /actual377, fp3065773916332326921:
927Accepted(+0.030397503683835092m) ->928terminal-rejected
(-0.0012186620250949076m) after20ms. Independent Stop928 accepts
+0.0038748126919154746m and actual derived387joins. The complete377original
wall and dynamic proofs pass, but horizon ends3.5207519184528335m/s, not rest.
Peer-only cross substitution keeps both outcomes in this new pair; old928/929
peer flip remains different-run evidence, not replaced. Anchor-only preserves928.

Final931 /actual and inspected387, fp9211360257116164304:
new complete physical evidence is present in publication, inspected and previous
accepted roles, while legacy solver-source status remains correctly missing.
Publication actual930/control10.089999777000001/cursor0.05;
387identity(snapshot9.909999778,problemfp8529508602331352197).
All three recorded physical wall proofs reproduce exactly, all diagnostic fields
match, zero solves. Original Stop ends0.0014241826052817652m/s against original
physical/rest tolerance0.007424263736913554. No original parent dynamic proof or
solver provenance is inferred. This is actual runtime evidence, not a rebuilt Stop.

Same387observed930Accepted ->931TerminalContingencyUnavailable:
now9.959999777 ->9.989999776; control10.089999777000001 ->10.119999776;
cursor0.04999999900000063 ->0.07999999899999999. Anchors are equivalent.
930full solved suffix terminal proof accepts+0.007091597519957693m;
931has clear wall continuation but current-world terminal peer proof rejects
-0.0003133950625793247m againstd2. Independent Stop930accepts
+0.008026033410105438m;931rejects-0.0006277465960242701m.
Ego control pose error0.02647991369482876 ->0.008850050742991812m;
control speed1.2316265797804724 ->1.1432223450777106m/s,
expected speed1.2314241826052823 ->1.1414241856052825m/s.
Both Requests use generation170, identical peer velocity
(-1.1461076103055168,1.0381153279723445) and radius1.9309999998882412,
with its position projected to their own now. Exact peer/time and ego epoch
sensitivity is the next investigation; no new producer fault is asserted yet.

## Timing and limits

D1/D2:9callbackwindows /364/366cycles, maximum20.668/26.215ms,
0/1overruns. D2overrun window wall1788929235.729720 is after firstD1Emergency;
not evidence for its cause. One causal-observation warning perDomain is startup
at0m/s (D1spawned541, D2before state520), not an active moving-race failure.

Both command streams369messages,40.03114179/40.01107481Hz, receiptmaximum
37.7714633942/37.6052856445ms,0gaps>50ms. Source duplicate/backward9/10,
gaps>50ms2/1, max94.999998/99.999998ms. Odometry453/451messages,
source0duplicates/backward and0gaps>50ms; receipt maxima277.849197/279.227495ms,
gaps>50ms2/1. Clock1811each; source0duplicates/backward/gaps>50ms; receipt
max274.181128/269.372941ms,1gap>50ms each. V2X arrays~17.14Hz, wire source
and bag receipt clocks are distinct; neither is the controller's receipt clock.
Throttled telemetry is not all-cycle evidence. Delivery continuity remains open.

Observation acceptance is achieved, full MPCC acceptance is not. Preserve the
new931source-free387pair and raw same-run inputs before an ego epoch repair;
compare architectures on any new formulation failure before another patch.
Moving multi-tick Stop to rest/restart, full intents, dev3/dev4, gates and
submission acceptance remain pending.

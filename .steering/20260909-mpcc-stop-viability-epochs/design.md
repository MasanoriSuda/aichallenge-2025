# Same actual Stop viability and observation epochs

Continue after4b2474da; no production changes selected yet. New sealed dev2
actual387Accepted930 ->rejected931 after30ms, unchanged peer170velocity/radius
and equivalent publication clock. Current-world wall continuation passes;
terminal peer+7.0916mm becomes-0.3134mm. Both exactRequests/actualartifact and
complete original physical wall proof are available and replayed without solves.
The original parent dynamic proof is not inferred from this evidence.

First compare exactly the same peer field and clock on the pair, and decode
same-run Odometry/steering/commands to identify each measurement source epoch.
The controller currently labels raw Odometry pose/speed with later decision time
before predicting0.13s; establish the exact causal inconsistency and a native
failing invariant before any repair. Treat prediction/model/measurement mismatch
and uncertainty separately. Preserve physical steering/response/yaw/velocity
joint epochs, raw safety monitor ownership and actual command provenance.
Do not change gains/noise/margins/limits/delays or weaken full rest/peer proof.

Compare sealed931A/B/C/D/supportStop before another formulation patch. All
failures are Unknown without physical infeasibility proof. Any selected producer
repair must include evidence, obsolete path removal, tests/build/replay/runtime,
spec/registry, local commit and explicit remaining full acceptance conditions.


Selected producer repair: make Odometry-derived state explicitly current before
feeding the unchanged actuator-delay predictor. An existing constant-speed /
constant-yaw-rate observation model propagates pose/yaw from source to decision;
body speed and yaw rate are held over that measured age. A stamped motion value
keeps all Odometry-derived quantities on one declared epoch. Then the existing
committed-command acceleration/yaw-response rollout spans exactly0.13s from now.
This is an estimator assumption for unobserved time, not measurement accuracy or
physical-ground-truth certification. Do not increase the actuator delay parameter.
Raw Odometry remains the safety-monitor/Recovery observation; steering's existing
independent resolver and timing questions are not claimed repaired in this slice.

Before-r2 exact application call-sequence native test has2zero-age passes and
4nonzero-age failures: at2m/s,5/15ms omit10/30mm; constant yaw omits0.00075/
0.00225rad. Before-r1 additionally tripped an unrelated6.09nm curved midpoint
quadrature error in its position assertion. Separate exact straight displacement,
curved yaw and time accounting in r2; no production/test acceptance tolerance
was increased. This is actual prediction/observer code with copied call sequence,
not an instantiatedROScontroller test.

Same-run matched930source age15ms /931age5ms. Existing constant-twist source-to-now
counterfactual (followed by unchanged entire actuator path viaSE(2)transform)
shifts19.149526/6.564913mm. Both reject:930terminal-0.000858296754m and931
-0.002501689556m. This shows old acceptance can mask the missing propagation;
it does not claim the estimator equals true motion or that this alone fixes race.
Peer-only and anchor-only preserve original930/931outcomes. Frozen931normal9arms
and supportStop4arms all solver-reject; physical feasibility remains Unknown.

Acceptance: stamped motion helper rejects invalid/backward/stale inputs; native
known-motion epochs and unchanged observer tests; source contracts;26package
build and package tests; exact source-to-now replay; fresh single/dev2 with
protected data restoration; no authority/model/limits/gain changes elsewhere.
Rollback baseline4b2474da. Remove direct old-epoch pose input to actuator rollout,
including its diagnostic missing-steering branch. Full acceptance remains open.


Consumer review found the canonical prefix start also matched the raw wall-monitor
pose. Keep independent current_execution_observation_pose_ for canonical prefix
and current-world physical snapshot, copied with the async snapshot. The raw
actual_wall_monitor_pose_ remains on contact/Recovery monitoring and historical
shadow diagnostics. The canonical prefix still must match its declared current
start and predicted endpoint; no identity check is removed or relaxed.

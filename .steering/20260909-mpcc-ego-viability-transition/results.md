# Ego transition and EKF calculation epoch

Baselineb19d6d7c. Whole MPCC acceptance remains open. No MPCC control, margin,
objective, solver, state model or publisher limit changes in this slice.

Exact934/935same376sensitivity:935withpreviousspeeds accepts/+0.003454998m;
withpreviousposes/progress accepts/+0.031201191m. Previous physicalsteering/
response or executionclock doesnotrescue; previouscommandgroup rejectssteering.
These hybrid requests diagnose sensitivity, not real state transitions.
Same-run Odometry/commands/steering chronology is preserved separately.
934Requestnow9.854999779usesposefromOdometry9.849999779. Controller treatment of
observation age is a separate unfinished issue; do not claim the EKF repair
alone corrects that consumer boundary or proves the entire934/935cause.

The installed EKF binary's real timerCallback has six clock reads. Controlled
native baseline: fixedclock passes;5msadvancebeforestoringepoch and before
publication both fail. State from9.98+dt0.02is10.000/x0.02, storedoroutputstamp
is10.005. Empty measurement queues and constantvelocity isolate the producer.
The probe's in-process Clock interposition is diagnostic/test only.

All13installed EKF headers match official1e90380a3570d02e16d40287e0adf51a27cf34d0.
Other retrieved2023versions do not match and are not substituted. Source clock
reads correspond to installed symbols/disassembly. The old launch differs only
in True/true spelling; the participant fork preserves installed lowercase form.
Sources, SHA256s and official URLs are in the downloaded upstream manifest and
participant UPSTREAM.json; LICENSE/NOTICE preserve attribution.

aichallenge_ekf_localizer replaces the underlay package in participant launch
and dependency. Same node/executable, topics/services/QoS/parameters. Callback
samples one time for dt, storedpredictionepoch, measurement delay, pose/twist/
odometry/covariances. TFuses the cachedpose timestamp. Filter math, noise and
smoothing stayunchanged; inherited clock-regression/reset behaviour is not newly
validated by the positive-advance tests. No runtime shim or base image edit.

26packages build42.7s; final lowercase-launch install rebuild4.92s. Build stderr:
setuptools deprecation; firstnewpackagebuild also gives future scoped-header
installation recommendation. No C++ compilewarning/error. EKF12CTestgroups /
102records0errors/failures,30cppcheck2.7wrapper skips. Fournewrealnodeclockcases
pass with one clock read, including queued measurements and observed Odometry
publication. Original seven mathematical test executables pass. MPCC60CTestgroups/
2372records0errors/failures/skips. Hostcppcheck has one unchanged falsepositive
for Eigen comma initialisation; the exact upstream source reports it too. It is
not suppressed or misreported as clean. No new test-source findings.

Same99recordedEKFinput/Odometryevents, sharedinitialstate and fixedper-callback
clock: oldinstalled vs repairedactualnode54ticks,2268state/covariancevalues,
maximumabsolute difference0, exactequalitytrue. This confirms mathematical parity
on those inputs, not reconstruction of all original hidden filter state.

Fresh single-r1 completed six laps in 255.7794189453125 s, penalty 0, with no
observed moving Emergency/Recovery. Callback maximum 15.460 ms, 0 overruns;
command receipts 39.9999 Hz, maximum gap 42.103 ms, none over 50 ms. One causal
observation warning was startup (spawned, decision 529, zero speed). Source
commands still have 12 duplicate/backward stamps and 3 gaps over 50 ms. Odometry
and clock receipt pauses reach 312.371 / 299.501 ms. Source/receipt timing is
separate; absence of throttled contract traces does not prove every-cycle joins.

Fresh dev2-r1 is rejected: first moving Emergency is D1 decision 937 at
1.3724878243982674 m/s, wall time 1788924827.449882713. No finish or result-details.
D1/D2 callback maxima 19.636 / 21.167 ms, 0 overruns; command receipts about
40 Hz, maxima 39.744 / 33.908 ms, none over 50 ms. Source commands retain 12 / 10
duplicate/backward stamps and 2 / 1 gaps over 50 ms. Odometry receipt maxima
277.653 / 282.086 ms; clock 281.079 / 280.086 ms. No causal observation warning.
Both campaigns stopped all containers and restored the protected user JSONs.

Both terminal and final 937 snapshots contain actual/inspected artifact 374,
world fingerprint 2962933974490556792, and its preceding ordinary Accepted
request 936. Four exact records replay with zero solves: 936 terminal peer
clearance +0.004820238906648067 m / independent Stop +0.007846802037691836 m;
937 terminal -0.0014817573213388169 m / independent Stop -0.0005679827768239054 m.
Original 374 is wall/dynamic certified, terminal speed 3.598851129056936 m/s
(not rest). Peer-only cross-substitution preserves both outcomes with the same
reported terminal clearances. It is a counterfactual, not authority.

936 observation/control epochs are 9.899999778 / 10.029999778 s; 937 are
9.929999778 / 10.059999778000002 s. Their published-plan clock anchors differ:
936 uses (10.009999779000001, 0.3594038730139186); 937 uses
(9.779999784000001, 0.12499999699999975). Thus elapsed artifact time advances
0.0255961189860816 s despite 0.030 s observation advance. The source and effect
of this anchor difference remain to be audited; it is not yet a proved defect.

The EKF producer repair is locally accepted. Integrated MPCC acceptance is not.
Next: exact 936/937 execution-clock ownership and controller observation-age
consistency, followed by causal repair/validation if demonstrated. Derived Stop
388 provenance from the earlier b19 run remains unknown and must not be filled
with a newly solved substitute. Moving Stop/restart, all intents, dev3/dev4,
gates and submission acceptance remain open.

Review and preservation: 746 artifacts are SHA256-sealed. XML, helper Python AST,
changed-document links and git diff whitespace checks pass. Registry validation
initially rejected older abbreviated baseline IDs; 79 IDs (including this
slice) were resolved through git to full immutable commits, with outcomes
unchanged and the mapping preserved. The strict registry now validates
93 snapshots / 204 experiments. No simulator is running. Protected user JSON
hashes match their original values. Required local commit follows this review.

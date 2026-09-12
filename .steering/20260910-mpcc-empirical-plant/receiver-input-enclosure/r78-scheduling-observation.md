# Owned controller thread scheduling observation

2026-09-12 JST. Production1241e0cf; documentation4998a704. Build117/tests104
remain the code validation. Authorized autonomous M4–M6; no push. Diagnostics
below cannot establish race acceptance and do not change any physical/clock gate.

R78 uses original standard runner with read-only20ms process sampling. All74
threads of the two owned MPC processes allow CPUs0–11. First D1 Ready920 fails
before-publication after a21.457mswall/12.600msCPU callback; no Start/laps.
The first analysis labeled counter reads by frame start. Docker discovery and
proc reads take time, so that per-callback bracket is invalid. Separately retained
r2 analysis chooses a before frame fully sampled before the approximate callback
and an after frame beginning after its diagnostic. In the resulting81.578ms outer
window, D1main records18.503ms CPU and14.541ms runqueue wait. This is not an exact
callback attribution. D1/D2 and worker threads use shared CPU groups across this
run. Observer9.502CPU-seconds in45.088wall-seconds adds load. All runtime stopped;
original DLL and user JSON restored.

Existing R36 main-only affinity failed; other-thread competition and inheritance
were unmeasured. New R79 is a distinct bounded diagnostic: use measured physical
core siblings0/6,1/7,...5/11; give each MPC main a different physical core group
and explicitly place every other existing/new thread of those MPC processes in
the remaining groups. Observer/runner also use worker groups. Verify actual masks
per thread, including newly created threads; unrelated host processes are untouched.
No priority/cgroup/kernel policy, controller parameter, rate or deadline is changed.
This tests owned-thread competition/inheritance, not a universal timing guarantee.
Original runner stops on its first failure and performs normal restoration; CPU
masks belong only to these temporary processes and disappear when they exit.
If masks were not established at the failure boundary, the partition result is
inconclusive. A pass alone is not permission to ignore other original failures.

Scripts and raw data are generated underoutput/20260912-nine-state-r78-scheduler-
observation, r78-scheduler-analysis and the separately named r79-role-partition.
Full M4–M6 and same final config/artifact acceptance remain open.

R79 completed with all74 thread masks assigned and verified before Ready: D1main
4/10, D2main5/11, other MPC threads0–3/6–9. It still fails D2Ready1438 at1.17m/s,
wp36, source948/job1433/index2, geometry821299624407504693. Current proof accepts;
nominal24.744999580, entry24.749999446, original deadline24.769999580,
before24.774999446. Callback19.431mswall/11.667691msCPU, current proof14.346890/
8.713429ms,32involuntary switches inside proof. No Start/laps. Reject this resource
variant for promotion; no persistent affinity or controller change remains.

R79's original counter bracket also overlaps its long sampling operation; r2
uses a169.622ms outer window, recording D2main31.760msCPU/31.025msrunqueue wait.
These broad windows cannot quantify the intervention's per-callback effect.
Other Autoware/AWSIM and unrelated host tasks could still use reserved groups;
this was only a verified partition of the owned MPC processes, not host isolation.
The observer consumed12.906CPU-seconds over54.899wall-seconds. Original DLL/user
JSON are restored and all containers stopped.

Next run original standard single-r3 on the unchanged production source to
validate the high-speed numerical-policy behavior not exercised by Ready-only
r77–r79. All M4–M6 and final-source acceptance remain open.
[Sealed runs, analyses and limitations](r78-r79-scheduler-evidence.json).

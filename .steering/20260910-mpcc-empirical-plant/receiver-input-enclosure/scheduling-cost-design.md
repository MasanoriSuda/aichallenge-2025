# Scheduling and residual cost after bb2d8843

2026-09-11JST. No overall completion. Baseline/rollbackbb2d8843.

Dev2-r34fails at movingD1decision941 (0.11m/s,WP29): nominal12.044999730,
before12.069999730,after12.074999730 versus deadline12.069999730. Steady callback
24.683162ms differs from30msROS advance. Decision946uses26.173907ms, including
primary20.386/applied14.673/continuation3.495/Recovery2.894ms. Four exact failure
inputs are captured. The existing25ms window rejects all these late events.

R216: unsafe FTZ/DAZ is a diagnostic counterfactual only and cannot be promoted.
Original four proofs are Accepted8.51–11.56ms; flushing saves about1.1–1.3ms but
invalidates the interval argument. It does not explain the whole live gap.

R217/r35: identicalD1r34decision941 input, binary and floating environment are
replayed at1Hz. During active startup control, the independent process's CPU
cost grows from about9–10ms to12–14ms, even excluding runqueue time. All20
loaded proofs remain Accepted. This low-rate diagnostic load is not acceptance.
Both running controllers'25loaded workspace files match the unchanged build,
including main38d8732f6edcfc5dced4bb85ca23e2517d062284659786221fc41a5c2acee6e0
and vehicle model908c8a691105fbe3d84b453dce527a3a9f8a3d95c8ea7f254abc86fe15d5fb87.
R35itself fails atD2793beforeReady andD1856afterReady. Its coarse.5sCPU samples
observe63/60CPU changes across74/73controller samples; all12logical CPUs occur.
These are observations, not exact per-callback migration counts or a causal
proof that affinity alone closes the deadline.

R218gprof500repetitions accepts all. Profiled time is not production latency.
The major visible groups are derivative maps, dual products and corner maps.
The anonymousbounds label aggregates both kernel and channel call sites in this
profile; do not infer1910history scans per proof from its combined call count.
R219revisits only exactsin(0)/cos(0) after the new force-matrix implementation and
new subnormal-cost evidence. It saves only0.2–0.6ms on current accepted cases;
unpromoted. R221typed constant products preserve every captured complete tube
bit-for-bit but save only0.1–0.4ms; unpromoted. No source changes followbb2d8843.

R220performs the requiredA/B/C/D comparison on the exact historical source278,
source decision930, inspected at finalguard941. Its original world and clocks
are not rebound to941. A is Accepted (54.752ms); B/C/D implementations reject
applicability because they support onlyOvertake intents. Their Cruise comparison
is inconclusive, not physical infeasibility or evidence of an architecture win.
The existing source already supplies a valid proof; the final publication cost
remains the failing boundary.

Next bounded diagnosticr36restricts only each owned controller's main thread to
one distinct physical core, selected from its existing allowed CPU mask and
sysfs topology. Other threads/processes, rates, priorities, physics, budgets and
proof guards are unchanged. Save PID/start-time, originalmask, chosenCPU,
topology and time of application. Restore a mask only if the same process is
still alive at cleanup; the campaign otherwise owns shutdown and artifact
restoration. This is an experimental scheduling configuration, not canonical
race acceptance or a submission change. Compare first failure and callback
cost before considering any production resource policy.

- [x] R34failure and r35same-input resource/load evidence collected.
- [x] R216unsafe counterfactual rejected for production; r219/r221 insufficient.
- [x] R220source architecture comparison; applicability limitations explicit.
- [x] R36controller-only andR37fullpartition comparisons failed; unpromoted.
- [x] Seal/registerR34–R39andR216–R222; local evidence checkpoint.
- [ ] Publication endpoint representation, longproofs and all remainingM4–M6.

R36controller-only affinity is insufficient: D2decision657beforeReady advances
3.754999916→3.789999915 (35msROS). The mask does not reserve the core from other
threads, and newly created threads can inherit their creator's mask. No affinity
policy is promoted. Both controllers exit normally and restoration records no
remaining live process.

R37tests an explicit resource partition on the samebb2d8843code: each controller
main thread gets one physical core, while its SMT sibling is excluded from the
campaign's shared pool. Every other thread of the three known campaign services
(simulator, Domain1Autoware, Domain2Autoware) uses that shared pool. Restrict work
to the current user's processes discovered through those exact container IDs;
record PID/TID/start-times and masks, handle newly created threads, and restore
remaining owned threads on cleanup. Other host processes are not in scope.
No priorities, limits, model assumptions or publication checks change. This
remains a diagnostic configuration, not a production deployment or acceptance.


R37also fails after resource partition: movingD11006callback15.424266ms advances
12.709999715→12.739999715, beyond12.734999715. MovingD11031postcallback18.829420ms
advances13.339999701→13.369999701. D21077isaftershutdownandnotfirstactiveevidence.
R37MCAPshows5msclockvaluesarrivinginburstsroughly16msapart. This is bag receipt,
not controller receipt. No CPU placement policy is promoted.

R222interposes only existingrclclockget/setfunctions, preserving arguments and
returns. A boundedmmaprecordsmonotonicstart/end,wallend,clockpointer,thread,value.
A20set/getsmoketestpreservesallvaluesand40events. R38globalservicepreloadcauses
Pythonlaunchsegfaultbeforecontroller; this is an instrumentation deployment
failure, excluded from controller evidence. R39scopespreloadtoMPCCnodeonly.
It records9743D1/9499D2events,0overflow; all recordedrclreturncodes are successful.
Read calls return the latest completed update. D11004,wall1789103903.907637175,
uses19.750574mscallbackwhileROSadvances12.319999724→12.349999723. D21006uses
18.632264msandadvances12.349999723→12.384999723. First moving failure isD1884,
25.595622ms; these later cases characterize timing, not acceptance. R39Ready/
shutdown phases remain separately available in the run. Neither trace proves
whether bursts form at simulator publication or transport; do not attribute
unobserved producer internals. Callback wall budget alone cannot authorize a
publication outside the certified ROS-time window.

All six runs stopped and restored original protected user artifacts. No new
production code, clock source, timer, QoS, priority, rate, safety or profile change.
[Sealed evidence](scheduling-clock-evidence.json). Next: measured structural
runtime/clock contract work, exact nanosecond endpoint and remainingM4–M6.

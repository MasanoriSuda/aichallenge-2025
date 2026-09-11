# Current physical proof cost, baseline 12c29078

2026-09-12 JST. Continue authorized M1–M6; no routine confirmation or push.
R51fails. First moving D1dispatch856/job855/source344 at0.37m/s: original
nominal9.234999818, callback9.239999793, before9.264999792, deadline9.259999818.
Current physical admission accepts; the final guard correctly rejects the late
packet. The actual first moving snapshot and same-decision phase log are saved.

Causal attribution from the new observer:
- D1current proof18.329957ms wall /11.332737ms thread CPU; request0.047669ms.
  Whole callback19.072ms /12.075332ms thread CPU. About7ms is off-CPU elapsed
  time (not claimed to be solely runqueue delay). Problem initialization0.203ms,
  pre-MPC0.049ms, Recovery0.014ms, publish/failsafe0.225ms. Source construction,
  Recovery and DDS are not the dominant producer for this first D1failure.
- Later D2moving979: current proof15.367608ms wall /11.039409ms CPU, plus
  Recovery4.197ms. This is a separate contributor, not substituted for D1.
- Both failures advance ROS25ms inside wall callbacks shorter than25ms. The
  original appointment's5ms initial quantization leaves20ms until its deadline.
  Wall elapsed alone is not the publication contract.

The expensive current proof is necessary because the latest pose is outside
its original earlier point population. R193/r194old-partition/certificate reuse
remains rejected; do not relabel, translate or retime old proof as current
without a separately justified complete theorem and new evidence. No grace,
positive hold, receiver/window/period changes, or suppressed failure is allowed.

Next bounded work uses exact R51current inputs with production authority frozen:
1. Reconstruct baseline current proof and profile its actual numerical kernel.
2. Compare an explicitly packed analytic force-Jacobian implementation. R224's
   old AVX2 generic J operators saved only~1ms and are not promoted. The current
   analytic force derivative still loops over7scalar columns, so vectorizing
   that distinct loop is a new implementation hypothesis, not repetition of
   unchanged rejected SIMD. Require exact result/boundary parity and portable
   dispatch before any production candidate.
3. If subnormal arithmetic dominates, compare mathematically outward interval
   coarsening as a separate unpromoted hypothesis. Never enable FTZ/DAZ or
   change the native plant's floating environment. Every new derivative bound
   must enclose the original/independent oracle; all native body/corner/rest and
   negative cases remain required. Do not assume this wins or preserves rest.
4. Off-CPU delay remains a contributor. Old affinity/resource partitions failed;
   no unchanged repeat or production resource policy follows without a new
   measured implementation/resource hypothesis. Cheap request/Recovery rewrites
   cannot explain away D1's measured current-proof cost.

No production numerical change is made yet. R51completed-run timing/phase
analysis and exact new native profile are next. CPU profiler overhead is not
production latency. M4–M6and local commits remain open; all-intent/Mission/
sibling/Store, actualStop/rest/restart, Recovery/Rejoin/Boost/async, same-final-HEAD
races/gates and same submission/eval remain required. Existing Stop trace
counters may refer to retired pre-C5 paths; audit their instrumentation against
scheduled source/index/actual body-rest evidence without loosening acceptance.

Completed comparisons, no production promotion:
- R306 exact D1/D2 inputs:111/87 current samples, medians6.752886/6.734115ms.
  Original source fingerprints and late rejection reproduce.100 repeats each.
  Gprof's instrumented D1median8.182358ms is not production latency; uninstrumented
  libraries and anonymous/inline attribution prevent assigning every symbol's
  sample percentage as an exact causal cost. Numerical derivatives/interval
  products/corners/trig are visible, not a proven single subnormal root cause.
- R307 generic SIMD5.899837/5.875887ms; packed analytic force5.834173/5.893930ms.
  All four complete original/current tube binaries match their baselines.
  Packing the additional force loop adds no useful gain. Reject for promotion.
- R308 explicit outward subnormal enclosure (integer-bit rounding to zero or
  signed minimum-normal, never FTZ/DAZ)8.091851/8.114444ms. Full tubes happen to
  match these two cases; slower, unpromoted. No universal validity claim follows
  from these two samples; no new coarsening in the production arithmetic.
- R309 same-map tire/zero trig sharing6.509531/6.498139ms, full bit parity.
  Adding exact zero subtraction/trig identities5.816996/5.738529ms changes tube
  bits while preserving these original accept/reject decisions. Unpromoted.
  R186/r189were revisited only because the C5retirement moved long source proof
  out of the callback and R51now isolates a short current proof. The new measured
  residual benefit is still insufficient to justify another standalone fix.

Completed R51logs: D1/D2normal507/428 (Ready221/152), pre-reject5/6,
post-window0/1. The later D2decision1074post failure occurred at wall1789139649.94855,
before shutdown1789139650.98122; preserve it separately from first movingD1856.
All callbacks are below25mswall, yet original ROS deadlines fail. Command receipt
means38.57/38.24Hz cover startup and shutdown and do not establish moving40Hz.
Original user JSON/DLL restoration is verified. No laps or M4-M6acceptance.

Next structural investigation: trace the original clock producer and control
appointment relationship before changing either. Installed rclcpp already defaults
ClockQoS to KeepLast(1) and an independent clock thread. Disabling clock progress
inside control would hide the raw-clock violation and is not a repair. Previous
R39trace confirmed freshest completed updates at get(), but did not locate whether
clock bursts originated in Unity publishing or transport. Numerical micro-fixes,
old point-population reuse, unchanged affinity and new deadline grace remain rejected.

[Sealed comparison evidence](current-proof-cost-evidence.json).

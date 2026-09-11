# Clock producer observation, baseline808830ac

2026-09-12JST. Authorized M1–M6continues; no production promotion or approval wait.
R51first moving current proof crosses its original ROS deadline. R306–309numerical
alternatives are unpromoted. Old R39interposition confirmed fresh completed clock
updates but did not identify whether bursts originate in the simulator or transport.

R310CIL extraction failed because the disposable container user could not unpack
compiler packages into its own/usr. No simulator modification. R311uses container
root only to extract already-saved compiler packages, then reads the original DLL.
Static CIL shows MultiDomainClockFanout.FixedUpdate creates a current timestamp and
publishes it to every registered vehicle Domain. UnityTimeSource reads timeAsDouble
on its original main thread. The installed rclcpp ClockQoS is already KeepLast(1),
with a separate clock thread. Neither fact alone establishes runtime burst origin.

R312adds two observation sites to a generated DLL copy: after timestamp creation,
and after each original vehicle-Domain Publish call. Fixed-capacity mapped records
store frame number, exact seconds/nanoseconds, Stopwatch ticks, UTC ticks and managed
thread ID. No console/file flushing per tick, no clock/command mutation, no delays or
rate changes. Original source/CIL, branches and exception targets are compared after
removing only the two observation sequences. All3690methods match; a20record smoke
check passes. Counter capacity200000must not overflow. This measures timing and is
not an acceptance binary or a submitted simulator change.

Next dev2-r52mounts the copied DLL and observer read-only only in its simulator,
plus the previously validated clock get/set interposer only in each MPCC node.
Save exact hashes, process/Domain identities and same-run earliest moving failures.
Compare producer frame/clock clusters to receiver updates and callback windows.
Do not change the simulator clock, slow time, disable its controller clock thread,
add grace, or relabel old proof. Original user DLL/JSON must remain/restored exactly.
Even a diagnostic race completion would not establish uninstrumented acceptance.

R52is inconclusive for moving timing. Source stops at frame5/sim99999997ns:
20timestamps/60records, then no source progress for>200wallseconds, zero normal
publications and no observation exception in captured Player.log. Main process
waits on a futex; gdbattach is unavailable in this environment, so deadlock cause
is Unknown.20startup ticks span frames2/4/5, including55mssim progress in29.795ms
wall in frame5. This proves startup source clustering only, not moving root cause.
The exact runner was interrupted and its normal cleanup restored protected files.

R53changes only diagnostic startup ordering: initialize simulator alone until its
observed clock passes the R52stalled source frame/time, then launch the original
Domain1/2Autoware services. This tests whether overlapping original initialization
with arriving controller commands contributed to startup stall. The original
Makefile, physics, observation code, clock/rates/windows remain unchanged. A120s
host diagnostic observation deadline is not a new controller timeout. Different
startup ordering is recorded and cannot itself become canonical race acceptance.
Use an isolated copy of the campaign and preserve its exact contents.

R53completes the intended moving observation. D1decision218/job217/source187,
actual0.11m/s, nominal9.304999799/entry9.309999791/deadline9.329999799/
before9.334999791. Current proof13.334507wall/11.125521CPUms; Recovery5.066ms.
MPCends atROS9.314999791; Recoveryends9.334999791. Source frame551publishes
three timestamps in2.817mswall; frame552four in2.777mswall about16.7mslater.
Same-run receiver updates follow those exact values. Source clustering is proven
for this moving interval; transport can still add delay. No clock policy change.
14361producer and93697/77692receiver events, all original rcl calls return success,
no buffer overflow. R313reproduces exact original/current physical proof and late
rejection (7.021349ms isolated). It reuses the unchanged R306compiled driver;
no new production build is claimed. R53overall callback overruns4/2, no adjacent
pairs; complete receipt means38.72/35.34Hz include startup/shutdown, not acceptance.
Startup ordering is diagnostic only. Protected user artifacts restored exactly.

Next: audit Recovery's requested current-wall observation versus unused normal-
state maneuver rollout. This is a separate contributor, not a substitute for
R51current-proof root cost. Full M4-M6and physical-cost structure remain open.
[Demand audit](recovery-rollout-demand-design.md), [sealed evidence](clock-producer-evidence.json).

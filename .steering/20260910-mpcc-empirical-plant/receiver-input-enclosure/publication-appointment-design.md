# Publication appointment ownership, baseline 3a96af57

Continue authorized M1–M6 without confirmation. R49 is failed. First moving
D1dispatch1021 (0.54m/s) tries original suffix index8 at ROS12.164999728,
67ns before its immutable nominal12.164999795. The source400/decision989,
original job1012, source/current observations and all8actual sends are saved.
No post-send violation in r49; existing r48actual late bracket still rejects.
The final guard correctly rejects early time. Removing that guard, rounding
67ns away, or shifting an already proved programme would mask the producer.

R300 diagnostic reconstruction used2s ledger retention and therefore failed the
actual-history equality gate. Its process returned0 but no dispatch candidate;
this is not a successful reproduction. R302 uses captured config's0.63s and
reconstructs all27current history entries,8source-tagged sends and original
input fingerprint2292506509033046902. Current evidence accepts in0.447289ms;
original early send rejects, exact opening and original+0..25ms samples accept,
deadline+1ns rejects. The+5ms clock sample is counterfactual, not a live send.
The current observation already belongs to the original full physical tube;
no independent cache or physical relaxation is used in this case.

R301 compares A/B/C/D on exact historical source400 without changing its world,
state, clock or constraints. A accepts134.726ms; B/C/D reject applicability to
Cruise. Those arms are inconclusive, not physically infeasible. A valid source
and current proof already exist; source architecture changes cannot justify
sending outside its publication window.

Scheduling alternatives on this sealed case:
- Existing25ms wall timer has no lower-bound relation to the ROS appointment.
  R49 provides the exact counterexample; r48 also starts close to window end.
- A periodic ROS timer shares units but its arbitrary initial phase still need
  not match a programme bootstrapped from a later actual Emergency publication.
- Use an absolute ROS appointment alarm, rearmed from the exact next original
  programme index. Bootstrap uses the actual preceding Emergency nominal+25ms.
  Worker and alarm must share one producer of this same next appointment.
- Waiting in the control callback for clock advance would occupy the single
  executor and complicate clock-stall safety. It is not selected.

Selected change: one ROS alarm opens each control callback at or after the
shared appointment; it never retimes an existing programme or issues commands.
Strict fresh context/generation, complete actual prefix/current physical proof,
wire/slew and original before/after/deadline guards remain the sole authority.
An already-late appointment invokes the normal strict checks without extending
its window; the next recorded actual Emergency supplies a new bootstrap.
Keep the existing25ms wall timer as a clock-stall/regression safety observer.
Use only the existing odom_timeout_sec for a stalled clock, and the existing
failsafe path with a new actual decision ID; it must never supply normal control.
Clock regression still invalidates observation/context/ledger through the
existing callback handling. A stopped ROS clock must not disable Emergency.

Retire the unconditional wall-driven normal callback in the same slice. Preserve
40Hz original nominal sequence,25ms publication window,250ms receiver profile,
130ms nominal origin, all solver/model/physical parameters and public contracts.
The alarm grants timing only; a late wake remains a failure. CPU scheduling or
DDS worst-case latency remains unproved and must be measured in a new run.

Required before runtime: real rclcpp timer tests for the saved67ns boundary,
1ns lower bound, callback rearm/cancel, backward clock jump, late wake without
retiming, and published25ms sequence. Validate shared next-appointment producer,
source104contracts, full build/package tests, source review and evidence hashes.
Then committed dev2-r50, followed by causal repairs and all remaining M4–M6.
Rollback3a96af57. No grace, positive hold, new timeout or alternate authority.

Implementation uses one `Alarm` with immutable requested ROS ns and bounded
one-shot ownership. A backwards internal timer rephase cannot fire before that
appointment. `next_scheduled_publication` is the former worker producer, now
shared with the alarm. A typed clock watchdog preserves steady timeout semantics
and supplies only a forced existing failsafe callback with its actual decision ID.

R303passes8real rclcpp timer tests; r304adds3steady watchdog tests,11total. The
first source check has6signature-lookup failures after `control` gains a forced
failsafe argument; update those anchors without weakening their body assertions.
All105source checks pass, including the wall observer's inability to invoke
normal control. Buildr90passes26packages; testsr78passes2545records/65groups
with zero errors/failures/skips. The next committed runtime trial isdev2-r50.

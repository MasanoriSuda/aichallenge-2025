# Scheduling revisit on short independent current proofs

2026-09-12 JST. Production ad18d262, build99 all26, tests86 all2570/66.
No source/parameter change or scheduling policy promotion in this comparison.

Standard r60 fails. First D2 pre489 before Ready: source486/solution6/index2,
current proof18.192mswall/10.052msCPU, domainOutside, seven involuntary switches
in proof and one in admission logging. R354 independently identifies lateral
position and heading outside the source domain; full pose symmetry still misses.

Stop trigger D1 post1074 in Ready: retained source1069/solution548/index4,
current domain accepted1.357mswall/1.355msCPU, no proof-phase preemption. A newer
source1070 is actual-prefix incompatible. Ready worker cost79.929ms/domain29.538;
retained source worker28.238/domain12.115. Callback7.379mswall/2.615msCPU has
MPC1.999ms, Recovery4.601ms, publication0.725ms. ROS time advances from15.019999664
to15.039999663 during Recovery. Its exact competing thread is not identified.
Original nominal15.014999667/deadline15.039999667; before15.039999663 and final
after15.044999663. Whole source rest15.274999667 covers both; the per-packet25ms
window does not. Raw command topic completes at15.039999663, but that topic is
not the final control command: final publish follows it and takes0.127680mswall.
Do not substitute the raw-topic clock, freeze the clock, or relabel this as the
previous terminal-rest horizon bug. Guard correctly rejects the actual bracket.

Later D1pre1115/index2 also has an accepted current domain; it misses deadline.
R354 reproduces all three original source/current identities, exact actual
history, slew passes and actual pre/post guard outcomes. Domain enlargement
cannot repair the two already-domain-accepted D1 failures. Completed bag timing
includes startup/teardown; D1/D2 cycles1724/1642, p99 values11.41774151/10.84012261ms, adjacent overruns0/0, no laps.

Revisit condition for rejected old R37 is met: that experiment used15–26ms full
current proofs on bb2d8843; current common proof is1.3msCPU and r57–r60 now expose
involuntary preemption in different phases. R61 will repeat only the existing
bounded CPU-partition arm on unchanged ad18d262. Two controller main threads get
separate physical cores; other threads of the same three campaign containers use
the remaining cores. Exclude the reserved SMT siblings from campaign workers.
Only same-user PIDs/TIDs discovered through exact owned containers are modified;
record start times, original masks, target masks, events and restoration. No
other host processes, priorities, clocks, DDS/QoS, control/model/solver/safety
parameters change. The observer itself makes this diagnostic only. Standard
race acceptance remains open regardless of its outcome. On observer exception,
interrupt only its owned runner so normal Compose cleanup/restoration executes.

Compare first failures, phase wall/CPU/preemptions, original publication clocks,
worker availability and fresh domain coverage. If it still fails with little
preemption, use that evidence for the next producer/architecture decision; do
not promote this policy or repeat it unchanged as a cure.

Completed outcome: r61 also fails and the CPU policy is rejected. All 30 owned
changed threads exited, restoration errors zero. See the
[availability audit](scheduling-availability-audit.md) for current-prefix evidence
and separate source programme comparison. No standard acceptance is claimed.

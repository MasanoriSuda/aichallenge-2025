# Stop alternate timing observation

2026-09-11 JST, baseline/rollback2c3d8d7c, dev2-r18, buildr41/testsr33.
Full completion remains open. The packet union repair passes its exact r17
replay; fresh r18first loses D1published Stop249 at decision839 and enters
moving Emergency843 at0.12m/s (wall1789066054.297481233).

At838 the synchronous alternate revalidation of source241 took229.759ms;
callback255.508604ms. Its outer reason is SteeringUnreachable, which also
masks deeper failure of a feedback bundle. That bundle can acquire full
authority when actually certified; it is not an obsolete diagnostic path
which may simply be removed. Later published Stop249 succeeded at838, but
the long callback left actual history from7.59499983to7.854999824:259.999994ms.
Exact839native replay rejects HistoryUnavailable under the unchanged250ms
profile, including after the correct upstream Stop helper/materialization.
The numerical packet union must not fill this real missing coverage interval.

The earliest remaining unknown is the producer cost inside alternate838.
Its exact request/result is absent from the saved final-authority snapshot,
which contains the later839/source249. Do not relabel that as838/source241
or replace its observation. Capture the first alternate exceeding the existing
publisher period, its immutable request and all measured proof-region times,
before the later Stop selection replaces it. Use the existing bounded private
snapshot recorder with one additional boundary bucket. Keep actual published
source distinct from inspected alternate source. This is observation only:
same solver, candidates, source, safety checks, profile, rates and authority.
All filesystem I/O stays off the control callback. Account for capture setup
in the existing snapshot timing region. No new timeout or runtime policy.

Extend the recorder's queue/dedup/shutdown test so startup/final/moving events
cannot consume the new overrun bucket, and it retains the first exact request.
Build/package tests, local commit, then a fresh diagnostic dev2-r19. The new
observation must lead to exact replay and causal repair of the measured work;
remove or retain it as a bounded diagnostic after that review, with no authority
promotion. No timing or race acceptance can be claimed from this instrumentation.

r18D2has a separate later loss996/source497 (previous995same source accepted):
full applied peer envelope rejects d1 at12.679999729. Preserve this world for
subsequent audit, not as the cause of earlier D1loss. r18callback p99D1/D2=
40.62503085/35.53703238ms, adjacent overrun pairs47/53 (startup/teardown included).
The run terminated at the first detected moving override, with no race finish.

## Local verification and clock distinction

Buildr42=26packages/4m39s. New bounded recorder case passes. Initialtestsr34
found one source-parser test tied to `failure_snapshot_ms =` rather than the
variable boundary; accumulation uses `+=`. The same before-overwrite/intent/
same-source assertions remain. Correctedtestsr35=2430records/62groups/zeroerrors,
failures andskips. No threshold or behavioral acceptance assertion was loosened.

At838 the command header is7.60499983 but bag receipt iswall1789066054.1960902;
the preceding receipt interval is262.683392ms. Next command header7.854999824
arrives9.541273mslater. The live history records actual publication time,
whereas the final packet guard compares the nominal decision stamp. The
empirical receiver analyses test ROS source age, not a guaranteed wall-time
transport bound. Keep these clocks distinct; the observed gap is real and
must not be filled by backdating a command. Publication timing remains part
of the unresolved acceptance, alongside the slow proof producer.

The diagnostic driver `replay_retained_cost.cpp` is compiled as nativer84.
Freshdev2-r19will provide the missing original slow input. This observation
does not fix runtime or establish acceptance. See
[evidence](stop-alternate-timing-evidence.json).

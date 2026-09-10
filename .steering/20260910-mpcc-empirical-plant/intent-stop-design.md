# Preserve the published source's stopping obligation across tactical changes

Baseline aec8d32e. Evidence is the frozen8bfdaa52 application-dev2-r1 D2
decision1294, worlddd81d6bd28141e1f. ShiftOut→FollowPrepare→Idle clears the
live tactical identity; ordinary source775 rejects Cruise as intent-mismatch.
Atomic admission reports previous source missing after tactical filtering.
The latest actual publication is source778 at1293, not source775. Its Stop
also receives the newly requested Cruise identity and rejects InvalidIdentity
before physical evaluation. That is the earliest broken Stop producer contract:
the previously published trajectory's stopping responsibility is confused with
the newly requested normal maneuver. The final Emergency masks that distinction.

Toolr29 exactly reproduces both source775 and actualpublished778 rejection.
On the same world/body/prefix/grid/peers/model/packet, retain source778's upstream
ShiftOut identity for the explicit Stop evaluation. It proves345samples/1.725s,
complete rest and full wall/peer clearance10.061819599m; materialization/rejoin
and production adapter succeed. Captured scalar progress is retained; native
body projection is used, not a reconstructed live waypoint service. A/rightB/C/D/G
also certify this world; leftarms and both direct complete-rest SQP clocks
reject. It is not physically infeasible. The missing previous-accepted1293
request remains missing; no observation from r12 is substituted.

Introduce a dedicated published-Stop evaluation which receives the last actual
publication's plan and causal PublishedPlan clock, binds only its upstream
identity, and calls the unchanged full Stop evaluator. Keep the bound Request
alongside its result for exact materialization. Generic normal evaluation and
generic Stop identity validation remain strict. Never bind an unpublished
candidate or extend its normal cursor. The current desired intent still owns
the next asynchronous normal problem.

The generated artifact explicitly requires terminal body rest. Its immediate
rejoin uses its sealed upstream proof intent, and output uses that same proof
identity with the existing certified-Stop publication role. A subsequent desired
intent cannot erase the last published Stop obligation. New normal authority
still requires its own current-world certification. No resumed/expired normal
Mission is authorized by this operation; no tolerance, extra hold, retry budget,
solver limit, receiver delay or safety margin changes.

Regressions cover ShiftOut→Cruise, source/proof/packet identity through complete
Stop materialization, rejection of unpublished/noncausal clocks, missing packet
prefix and changed wall. Reproduce the old producer first. Then build/package,
tool replay of actual778 and negative r12, fixed-commit ordinary dev2. Record
all results and timing; M4–M6 stay open until integrated acceptance. Rollback is
the focused commit for this slice based on aec8d32e.

Buildr29 passes26packages/4min14s. Testsr23 runs2407records: the newnative
transition regression passes. Two source-structure checks still refer to the
old local struct and direct evaluator name, producing3colcon failure records
(oneaggregate plus twoassertions). Update those references to the shared type
alias/dedicated evaluator. Keep all forbidden publish/store operations and
Stop publication-role checks unchanged. No failedrun/test is accepted.

Final local evidence: testsr24 all2407records/62groups pass in29.07s;
toolr30 invokes the actual new publishedStop API. Original1294published778
returns345samples/+10.061819599m/completeRest, bundle/rejoin/adapteravailable,
acceleration-3 and physicalsteering-0.0385987374425 exactly match its serialized
prospective packet. Independent933source424 still rejects peer-0.005110710497m,
with no bundle. Freshdev2-r13 is next; neither replay grants runtime authority
nor closes input-application uncertainty or M4–M6.

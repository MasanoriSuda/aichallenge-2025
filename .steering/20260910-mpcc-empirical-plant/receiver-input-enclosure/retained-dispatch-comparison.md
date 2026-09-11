# Retained dispatch and worker comparison after r56

2026-09-12 JST, baseline/rollback4495ab3e. Audit/diagnostic phase; no production
change. This follows two starting-domain production changes, so compare
architectures on the frozen failure sources before any third production patch.

R56 earliest recorded pre-send failure: D1decision530, source14/job525,index4,
Track before race session. First readyjob528/source19 is actual-prefix rejected.
Retained current proof passes (PoseMismatch independently reproved), but actual
before.854999980 exceeds original deadline.849999991; nominal.824999991,
entry.829999981. Current proof6.962911mswall/5.199803CPU; full callback25.644mswall/
5.950993CPU; MPC25.359ms and Recovery.041ms. There is about18ms outside the
reported current proof/request intervals inside MPC. Off-CPU/blocking attribution
is Unknown; do not call it solver cost, CPU runqueue, logger or Recovery without
further observation. First admission log and second current-proof log are about
25ms apart. Preserve phase/clock distinctions.

R56 run termination trigger: D2post1268, retained source742/job1263,index4.
Readyjob1266/source742 is actual-prefix rejected (worker25.451ms, domain13.026638).
Retained current proof13.265656mswall/6.484779CPU; oldworker16.790/domain8.579372.
Current tire mismatch is independently reproved; domain_use=UnsupportedSuffix.
Nominal20.604999553, entry20.614999539, before20.624999538, after20.634999538,
deadline20.629999553. Final publish2.963365mswall vs rawpublish.009017ms;
Recovery.092ms; callback17.386mswall/7.704060CPU. Do not backdate these events.

Completed whole-run counts (include startup/teardown, no moving laps): domain
accepted143/91; firstpacket outside34/17; unsupported retained479/464 (D1/D2).
Domain coverage improved but live acceptance failed. Later pre-send rejects can
also have clocks inside the packet window: original rest/slew/identity guards
must be inspected individually, not all labeled late. Preserve first teardown
signals and separate events after them.

Required comparisons:
- Existing persistent A / stateless B / rough C / bounded offline D on the exact
  historical source snapshots of firstD1pre530 and D2post1268. Recheck fingerprints;
  unsupported intent means inconclusive. No solver infeasibility claim.
- Exact current replay for both failures, authenticating four actual preceding
  suffix sends and original source IDs. Existing index-zero replay is insufficient.
- Same full original programme/domain/current world: baseline first-windowD;
  explicit independently computed observation-time domain across original indexed
  packet windows (past published packet memory retained); combined source-prefix+
  independent-domain certificate eliminating duplicate future propagation; and
  a combined generalized variant if numerically feasible. These are hypotheses,
  not new authority or permission to increase any actual publication window.
- Observe the earliest unexplained MPC/off-CPU phase if it remains causal. Do not
  suppress diagnostics or alter scheduling/affinity without evidence; prior
  R36/R37 resource partition failures remain history with their revisit conditions.

All proposals preserve original25ms per-packet pre/post windows,250ms receiver
model,130ms control origin,100ms steering delay, physical limits and budgets.
Old point-future reanchoring/reuse (R193/R194) remains rejected. A generalized
set/time theorem must be recomputed from every declared starting state/time and
retain the exact original input memory, plus full current world and actual prefix.
M4-M6 remain open; no unchanged runtime rerun until a meaningful validated change.

R332:exact historical source14/source742 both persistentAaccepted. Existing B/C/D
producer supports Overtake only; Track/Cruise arms are inconclusive. R333old replay
assumed growing history and failed before checks. R334restores actual256transaction
capacity and config.5+.13s history retention, authenticates all4actual sends and
exact pruned current history. Captured independent physical hashes match both.
D1pre530reject and D2beforepass/afterfail reproduce; current proof medians4.364/3.968ms
in isolation. [Evidence](retained-dispatch-evidence.json). Next compare complete
source/retained theorem structures and observe the unexplained off-CPU phase.
No production change in this audit slice.

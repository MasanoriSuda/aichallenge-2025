# R27 complete Stop timing audit

2026-09-11 JST. Baseline and rollback: `4a4b4148`. The directional wall commit is the rollback baseline. The bounded synchronous
materialization reuse below is implemented and locally verified; fresh runtime,
M4–M6 and live25ms are not complete.

Dev2-r27's monitor first detects D1decision848, .11m/s, WP29, wall time
1789082846.042177942. Earlier causal evidence is D1decision842's317.497992ms
callback, followed by843's loss of authority. The latter snapshot has current
predicted u=.2052082105m/s although its emitted measured-speed trace rounds to
−.00m/s. Do not relabel848as the first internal failure. Stop/normal execution
after that point and later D2ShiftOut are not integrated acceptance evidence.
The monitor begins shutdown at1789082854.6672575; distinguish later events
and teardown. Protected result JSONs are restored.

## Exact chronology

At842, the next requested normal intent isCruise; the available solved Stop121
retains its originalTrack source identity. The captured alternate request is
Track, now7.609999829/control7.739999829, currentu=.0037175167703572733. The
previous actual publication is839/source120, distinct from inspected121. The
exact previous accepted request is unavailable and is not inferred.

The121Stop fully certifies, then materializes as128 and publishes acceleration
1.2517242431640625, steering wire.4924049973487854. Live regions:
primary.119ms, Stop-lattice134.606ms (applied proof126.463ms), output-successor
177.421ms, callback317.497992ms. The actual history has packets at7.609999829
and7.924999822: gap.314999993s, exceeding the original.25s profile.

At843, ordinary128revalidation isintent-mismatch. A separately generated
publishedStop nominally accepts/materializes129, but fulljoin rejects. Atomic
admission holds the external Stop role; the final publisher emits Emergency.
This lower-layer rejection cannot be repaired by changing the intent label,
backdating the315ms gap, or extending the250ms profile.

## Replay and comparisons

`output/20260911-nine-state-dev2-causal-r27` preserves the exact842request,
same-world ordinary architecture arms and complete-restY comparisons. A accepts
143.071ms, Cleft106.451ms, Dleft153.18ms. Bleft/Bright/Cright/Dright/Gleft/Gright
reject. Y's first short candidate rejects; its existing5s second candidate
nominally accepts27.3494ms. These are nominal results without applied authority;
all-method infeasibility is not established.

R149reproduces842/source121without changed inputs: fullproof100.815207ms,
applied95.012935ms,191packets,1071checked intervals, maximum5body partitions,
rest12.959999829,peer+.150481967m. Same-program prediction body-only67.934075ms;
body+corners85.041146ms. Materialization10.972927ms; fulljoin100.240558ms,
including94.401526ms applied proof. Both heavy computation and recomputing the
same current-world response contribute; removing just the duplicate would
not meet25ms. Original source/packet/model/margins and all proof gates remain.

The generic843loader initially aborts with`original native replay failed`.
R150rebuilds actual128from exact842/source121 instead and compares all captured
artifact/physical fields. Only`physical_proof.completed_sec`differs. It reports
comparison failure (exit3), preserving that result. Generic native adapter
rejects`InvalidElapsedTime`atstage939; this is a diagnostic-loader limitation
for this materialized programme, distinct from live843's history rejection.

R151records both completion clocks (captured7.757136017, regenerated7.749387819)
and excludes only that new computation timestamp from its second comparison.
Every artifact, trajectory, original programme/provenance, physical outcome,
identity and world field matches. Under the original source intent, the exact
843request rejects applied input prediction reason5 (`HistoryUnavailable`).
The original programme's remaining suffix also rejects5 with the same315ms gap.
The unmodified requestedCruise result remainsintent-mismatch. No captured
source is silently relabeled and no output artifact is edited.

R152profiles the same original prediction with diagnostic`-pg`instrumentation;
ten repeated inputs provide samples only, not additional acceptance trials.
The interval Jacobian product owns55.72%self time (14,280,762calls), derivative
and centered propagation7.96%each. Trigonometric functions and allocations are
minor by comparison. Instrumented timings are not live or native acceptance.

R153attempts an isolated numerical namespace to compare structural derivative
activity, but its first baseline compile fails: the override header also reaches
the unrelated footprint translation unit. No numerical conclusion. R154uses a
separate diagnostic include name; original and modified internal Jacobian types
cannot mix across shared-library ABI. Its comparison completed: isolated baseline
125.530562ms/fulljoin125.061414ms, sparse103.681711/103.205389ms, sparse with
point-interval multiply103.322104/102.941923ms. All accept the same191commands/
1071intervals/rest/whole proof. The latter two change54,028stored scalar bounds,
maximum absolute difference4.440892098500626e-14; they are not bit-identical.
Their two complete565,488byte tube dumps are identical to each other. This
roughly17%reduction is insufficient for25ms and has not been promoted. It still
needs independent native inclusion and protected derivative-activity metadata.

R155direct natural interval propagation, without mean-value Jacobian bounds,
rejects state bounds at7.799999829 (6.719383ms); no production authority. It is
not an adequate replacement for this original programme. No fallback or
physical threshold was added. R156tests borrowing the already certified tube
only during exact same-world materialization while rerunning all original
physical checks. The diagnostic join accepts in 15.200193ms with all 1071 physical checks.
Its global diagnostic pointer is not a production design and was not adopted.

## Open implementation boundary

Investigate exact same-program numerical cost and immutable same-world proof
reuse through Stop materialization. Retain the original delayed-braking solved
Stop, every input variable/response, physical margins, history coverage,
full-rest horizon, peers/Follow, and final identity/packet guards. An earlier
braking shortcut or stale proof must not conceal this long-programme failure.

Any sparse derivative implementation must prove that inactive derivatives are
identically zero, protect activity metadata from unsynchronized writes, and
preserve conservative derivatives for all active/nonsmooth branches. Explicitly
record any removal of artificial rounding width from zero-minus-zero; do not
claim bit identity without comparing the actual complete ranges. Validate
native inclusion, sign/rest/launch/steering boundaries, original positive and
negative worlds, fullmaterialization/join, build/package and fresh runtime.

Actual publication-clock alignment remains separately open: final packet checks
use the nominal decision clock; history records actual publication. A faster
callback alone does not establish that clock contract. Fresh same-HEAD campaigns,
all intents/Stop/restart/async, gates and same submitted tar/image/eval remain.

## Implemented synchronous materialization join

The earliest producer of the r27 history failure is the slow synchronous Stop
proof, with an additional identical numerical propagation after materialization.
This slice removes that duplicate computation only. It does not fix the initial
101ms proof or actual publication-clock alignment and does not close M4–M6.
Rollback is `4a4b4148`; the original full predictor remains the mismatch path.
No new normal authority, retry, hold, grace, rate, timeout, margin or input-profile
change is introduced. Earlier A/B/C/D/Y results above remain nominal comparisons.

`evaluate_materialized_stop(request, certificate)` borrows one immutable parent
certificate only during the call. The controller passes it only from the current
accepted Stop to the just-materialized Stop. Request/proof objects do not retain
the parent certificate or create a cache/parent chain. New nominal trajectory,
source, decision, cursor, first-packet and production-identity checks still run.

Reuse additionally requires a terminal artifact whose immutable provenance names
the parent's solution/problem, exact programme/observation/profile/model context,
rest time and velocity ceiling. Decision/time/current world/footprint and physical
wall clearance must match. Publication and every original future swept sample
are checked again for state/speed bounds, complete wall cells, peers and Follow.
Any mismatch executes the original predictor. The new private certificate owns
the materialized plan and its nominal fingerprint; the parent cannot authorize
that plan by itself. The final command equality checks are unchanged.

Changed production files are the applied-program header/implementation, retained
revalidation header/implementation, and controller. The controller's materialized
Stop log now records `numerical_tube_reused`. Two behavior tests cover complete
range equality with independent fresh prediction, parent ownership, a later
decision, missing history, a newly overlapping peer and changed input profile.
Existing physical/history/identity/progress regressions remain unchanged.

Buildr55 passed. Testsr45 exposed one existing source-name contract failure,
reported twice by the test summaries: a new generic helper was named like the
historical deleted progress guard. It was renamed `evaluate_stop_candidates` to
state its actual role. No progress logic or source-contract test was changed.
Buildr56 passes 26 packages (9.36s); testsr46 passes 2453 records / 62 groups,
zero errors/failures/skips (29.80s).

R157 rebuilds prospective packets and prefixes using the same node functions.
The regenerated control pose is exactly equal, so the original course projection
is retained explicitly. Full reused/fresh tubes, including every publication,
endpoint and swept body/corner bound, are bit-identical in all three worlds:

| Original world | First full proof | Reused join | Independent fresh join | Physical checks |
|---|---:|---:|---:|---:|
| r27 D1 842 | 101.230731ms | 15.123690ms | 101.293818ms | 1071 |
| r26 D1 928 | 13.608181ms | 2.246395ms | 12.650230ms | 172 |
| r23 D1 1018 | 23.480045ms | 4.438527ms | 22.724535ms | 374 |

All joins match both their current request and nominal proof and produce the
existing production-adapter authority. These offline results are not permission
to publish diagnostic candidates. Native159–161 also completes all 18 compile/run
commands: earlier positives, actual r25/r26 publication joins, signed-source Stop,
real r18 history-gap rejection and later peer/turning-wall negatives are preserved.

R158 tests unchanged arithmetic with alternate compiler schedules. Isolated O2
baseline 123.910090ms, forced product inlining 127.558659ms, all-Jacobian-operator
inlining 135.281460ms and O3 100.883914ms all preserve every stored tube bit. The
production library already uses O3. No compiler/ISA/fast-math or derivative change
is adopted. These trials do not solve the initial long-proof cost.

[Evidence manifest](same-world-stop-evidence.json) freezes the diagnostic failures,
source inputs, component results, native regressions and build/test logs. Fresh
dev2-r28 is next; original acceptance and failure monitoring remain unchanged.

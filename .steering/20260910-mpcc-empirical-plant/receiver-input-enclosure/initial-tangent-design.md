# R69 physically connected initial tangent

2026-09-12. Baseline/rollback d6c5de15. Authorized M1-M6 implementation and
necessary local commits continue; no routine approval gate. Overall acceptance
remains open. This slice does not change production until its causal comparison
and failing regression are established.

## Evidence and scope

Standard r69 starts, moves briefly and stops without laps. Source supply status
shows drafts available, normal submissions/worker completions/mailbox advancing,
no worker exceptions, and accepted Store sources stopping at D1 308 / D2 462.
These separate mutex reads are not atomic job transactions. Old geometry is
correctly rejected; no source cache, context exception or replenishment rule is
justified. Exact old-geometry D1 source316, new-geometry D1 source350 and
old-geometry D2 source320 are distinct. D2 source320 does not explain the
uncaptured later new-geometry source462 failure.

R416 sealed A/B/C/D comparison accepts C/D on two of the three sources, but all
arms reject new-geometry source350. R417 original QP cold/warm replay rejects
all six trials under the original 4000-iteration budget. R418 HiGHS numerical
linear feasibility: source316 and350 are linearly infeasible; source320 is
feasible with independently checked residual2.57e-10. R419 source350 prefix1-7
is feasible, prefix8 first infeasible. Removing only lateral state boxes or
input boxes makes this linear system feasible. These diagnostic ablations never
produce candidates or alter production bounds. They are not nonlinear physical
infeasibility certificates. HiGHS status4 in two other ablations is inconclusive.

The initial adapter builds each tangent from an unrelated future desired state:
source350 actual velocity0.2702067m/s jumps to the soft9.8534619m/s reference at
stage1. Auxiliary body states are also advanced using these disconnected speeds.
The unchanged initial affine QP fails before its normal nonlinear refinement can
correct the trajectory. The failure is in the numerical initialization producer;
solver rejection detects it and subsequent certified-source loss is a consequence.
Emergency and incompatible-source rejection remain required safeguards.

R422 copied-adapter comparison changes only initial tangents, retaining all
original costs, bounds, timing, solver budgets and physical checks. Propagating
from the immutable initial nine-state origin makes all four new seed variants
solve and pass the source350 physical path. Source316 still rejects its dynamic
plane; source320 only the braking tangent solves. These remaining failures are
not fixed or relabeled. R423 repeats the complete architecture/terminal-proof
comparison. R420 wrong diagnostic member names and R421 missing context reseal
are preserved setup failures, not solver results.

The earlier rejected coherent-tangent experiment may be revisited because r418/
r419 now supply new causal formulation evidence on an exact initial QP. No
claim is made that all historical scenes are fixed by a coherent seed.

## Intended repair and acceptance

Separate the soft objective trajectory from initial dynamics. Build one native
nine-state trajectory from the original physical state under bounded reference
acceleration and a held steering tangent. Progress follows the native projected
forward motion within the unchanged virtual-speed/course domain. Select tangent
inputs within the already inset actuator and cumulative steering bounds. This
is an optimizer initialization only: neither this unsolved trajectory nor its
inputs have execution authority. Original costs/references, state/input bounds,
wall/peer/terminal constraints, SQP schedule, current-world acceptance, receiver
250ms, publication25ms, control-origin130ms and steering-delay100ms are unchanged.

Remove the obsolete disconnected per-stage reference tangent construction.
Continue to preserve semantic references for objective construction. All later
relinearization, exact physical proof, applied-program certification and the
single publisher authority remain unchanged. No old solution or point-future
proof is retimed or reused.

Before production: two native RED regressions show soft speed changing the
initial body trajectory and inconsistent affine/native transitions. The repair
must make them pass, retain adapter/model/package negative tests, reproduce the
frozen source350 result with the built production library, and preserve unrelated
rejections. Then full build/package tests and a committed standard dev2-r70 check
real source availability, callback/publication timing and both vehicles. Source
availability alone is not integrated completion. Same-final-HEAD repeated
single/dev2, dev3/dev4 six laps, all intents/Stop/rest/restart/Recovery/Rejoin/async,
gate1-3 and same submitted tar/image/eval remain required.

R425 both native regressions fail the original producer; R426 all22 adapter
regressions pass the repair. R424 is a diagnostic gtest include/link version
mismatch, preserved separately. Build110 all26 pass in13.5s. Tests96 records2587
report2 failures for one native test (CTest wrapper and native result): its old
prepared candidate physically failed, while the new initial tangent's prepared
candidate passes the exact physical adapter. R427/R428 confirm original heading
-0.5 and -0.51 accepted; -0.52 through -1.0 reject in the QP, never execute.
Update that fixture to require finite/constraint-valid/physically accepted
artifacts at the original state and no artifact at -0.6. The existing independent
nonlinear wall-departure rejection test is unchanged. This is an observed changed
optimum, not weakened bounds or acceptance of a failed test. Package rerun and
built-production replay remain pending.

Final validation: build11126packages/19.4s; tests972587records/66groups,
0errors/failures/skips, source105/C5 161. R429 uses the built production library
and original immutable source fingerprints: new-geometry source350 persistent A
passes the complete candidate/terminal proof; the other source A failures remain.
No acceptance is granted to diagnostic seeds, failed QPs or old point proofs.
[Sealed evidence](initial-tangent-evidence.json). Next: committed standard r70.

Code owners: `mpcc_rate_resolved_adapter.cpp:build` removes old reference-state
linearizations and builds the connected native sequence after all original
bounds/costs. Added regressions are in `test_mpcc_rate_resolved_adapter.cpp`;
the independently observed prepared-candidate change is recorded in
`test_mpcc_rate_resolved_shadow.cpp`. No normal authority was added or removed.

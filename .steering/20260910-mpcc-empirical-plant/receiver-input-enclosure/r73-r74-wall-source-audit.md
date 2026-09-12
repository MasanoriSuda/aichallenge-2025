# r73 publication timing and r74 source wall audit

2026-09-12 JST. Baseline `6c7c9615aff3e371ff06d58beff353d459b70478`,
build114 (26 packages), tests101 (2592 records / 66 groups / source105 / C5 161).
Authorized autonomous completion continues; no push. This audit has no production
control change. M4–M6 and integrated race acceptance remain open.

## Separate run boundaries

Standard `20260910-nine-state-dev2-r73` was rejected and automatically stopped
at D2 Ready decision529, before racing. Nominal1.009999987, entry1.024999977,
before1.029999976, after1.044999976, original deadline1.034999987 seconds.
The current physical proof accepted (domain_use8); its wall/CPU cost was
13.677627/8.399466ms. Callback18.325ms, final publication1.035688ms wall time.
The ROS clock advanced15ms across that final call. Scheduling/delivery and proof
cost remain distinct contributors; this is not evidence for increasing the25ms
window. The later decision602 before-window failure is separate.
No laps. The runner's zero exit code means completed failure teardown, not pass.

`20260910-nine-state-source-observation-dev2-r74` is a separately named,
investigation-only bounded continuation after recording unchanged controller
failures. The standard acceptance runner and all controller guards are unchanged.
The diagnostic stopped explicitly; both cars eventually stopped, no laps.
It does not establish race acceptance. At D1decision2138/D2decision2126 the
source generations remained valid, but the Store stopped at535/570 and the
latest worker outcomes were solver rejection. This does not reproduce r72's
advancing D1Store/generation-filter hypothesis and does not falsify it in r72's
different immutable world.

Both runs completed teardown and restored the original DLL and user result JSON.
Per-run rosbag analyses preserve startup and final-window coverage limits:

| Run | D1 callback maximum / overruns | D2 callback maximum / overruns | Command source Hz D1 / D2 |
|---|---|---|---|
| r73 | 25.099162ms / 1 | 28.957767ms / 1 | 39.069 / 39.726 |
| r74 diagnostic | 56.634418ms / 6 | 30.894326ms / 2 | 35.747 / 35.839 |

r74 adjacent-overrun pairs4/1; command maximum source gaps45ms both. No duplicate,
backward or nonfinite published commands observed. Topic publication and receipt
are not simulator application evidence. No average rate is a deadline guarantee.

## Frozen solver sources

D1source618 / decision1120 / snapshot16.014999642 / geometry7297210173200458920
is a complete wall-refinement failure. Exact body rest u=vy=r=0,
ey0.9038521327663225, lag−0.06170756260814941, heading offset0.1173935542592095.
The original failed QP has249variables/578rows and no dynamic-obstacle rows;
its active peer context does not make that first failure a peer-QP failure.

D2source2967 / decision3449 / snapshot82.714998151 / geometry13138610705333213011
was captured during shutdown with an interrupted500-iteration original solve.
Its immutable input can be compared offline; the interruption cannot establish
a live numerical failure or physical infeasibility. The earlier observed D2
geometry3957399223715686782 is a different world.

R448 rebuilt the current comparison driver and ran A/B/C/D/G, nine arms per
source. All18 rejected numerically. R452's existing restoration H also rejected:
D1 at the same wall boundary, D2 at a later physical dynamic plane. No full
certificate or normal candidate was accepted. All-method failure remains Unknown
without a bounded physical infeasibility certificate.

R449 HiGHS exact original linear rows: D1 infeasible; uniform physical Phase-I
slack0.0006210481270777898. D2 feasible with maximum residual1.3523199920988367e−10.
These statements concern the captured affine problem, not nonlinear vehicle
reachability. R451 omission diagnostics localize D1 to progress-wall rows458–497:
removing those rows gives feasible retained residual3.68e−10 while violating the
original rows by0.20480685. Omission is not a candidate or production proposal.
Removing future progress boxes alone still fails; lateral/lag omissions return
HiGHS Unknown and must not be reported as infeasible.

R450 confirms both exact current physical footprints clear under the original
0.2m lateral clearance. Its generic D1 QP replay used the wrong preconditioning
policy: `replay_recorded_qp` hardcodes RowToleranceNormalized, while the captured
wall solve uses internal equilibration10. That replay is not a production-policy
reproduction. R452 selects the serialized original policy: D1 warm replay matches
original primal0.014396660486333408 / dual8.98037104613064 and4000-iteration reject.
Cold replay also fails. D2 cold and warm solve the saved pre-interruption QP offline;
the full H pipeline still rejects its later dynamic problem. No solver budget,
tolerance, policy or input was changed in production.

## Producer localization and current hypotheses

Controller `mpc_controller_cpp.cpp:22330` constructs the current physical interval
via `find_cached_physical_wall_envelope` (`:35129`), then the canonical profile
(`:22455`) feeds `build_progress_wall_refinement` in `mpcc_rate_resolved_shadow.cpp:92`.
The latter already uses solved progress, profile segment slopes and a connected
native initial trajectory. Its artificial lag/heading pose boxes were already
retired from normal authority; proposing their removal again is obsolete.

D1's first profile upper bound0.8609313490270555 excludes the actual current
ey0.9038521327663225, although the initial QP state is correctly pinned and the
actual footprint is clear. The final failing wall row497 is stage20
`ey + 0.058434983985050135 * progress <= 0.911941…`.

R453 reconstructs the original current pose exactly from source course/lag/ey/yaw.
Zero-lag with exact heading is also clear. Both actual-lag and zero-lag variants
with the controller's0.025rad heading bucket and its geometric bucket guard
contact2cells. Thus omitted current lag alone does not explain the current
interval exclusion; the bucket's conservative pose enclosure is a demonstrated
contributor. Diagnostic lateral scans are sampled comparisons, not continuous
clearance certificates. All21 recorded warm-primal stage footprints are clear,
but those affine states are not an exact nonlinear/swept/dynamic/terminal proof.

Next compare only an independently measured current-heading interval in the same
world, resealing changed candidate identities and retaining every full proof.
Do not lower physical clearance, bucket widths, boundary guard, solver budgets,
publication periods or tolerances to force acceptance. A current-pose clear point
alone does not prove that a forward trajectory exists.

R454 measured the exact current-heading clear/occupied boundary in the unchanged
world (0.9225896697843459 / 0.9225896697959874m) and preserved the0.001m boundary
guard. Replacing only the first upper approximation with0.9215896697843459m,
resealing the candidate and running all nine A/B/C/D/G arms still rejects at
stage20 wall row497. Thus the current-envelope bucket exclusion is insufficient
to explain or repair this QP. This candidate is rejected and not promoted.

R455 next tests a different local representation: each future stage's lateral
interval is measured at its solved course/lag/heading in the same map with the
original hard-clearance footprint. The diagnostic retains full nonlinear,
swept-wall, peer and terminal proof. A sampled stage interval alone remains an
optimizer approximation and cannot become control authority. Current production
and all runtime configuration are unchanged.

[Sealed r73/r74 and R448–R454 evidence](r73-r74-wall-source-evidence.json).

## Follow-up architecture outcomes R455–R460

R455 pose-specific future intervals solve the D1wall QP; the next dynamic QP
still reaches4000iterations. The approximation has no full physical certificate.
R456 applies the existing exact-QP KKT comparison to that saved later problem:
the numerical solve succeeds in3.468ms, but the external trajectory adapter rejects.
Its detail is not a wall or terminal proof and is not promoted.

R457 runs the complete fresh-context A pipeline with pose-specific wall intervals
and the independently equilibrated solver backend. D1 reaches artifact construction
but rejects the initial lateral corridor: the canonical current-map upper0.860931
still excludes ey0.903852. D2 still fails its later dynamic plane. R458 makes the
current profile, semantic state0 corridor and initial terminal map agree with the
same exact-heading boundary. D1 now solves the full numerical pipeline, then the
unchanged exact wall proof rejects stage149 / waypoint31 / contact1 at world pose
(89629.564,43129.541,2.415). D2 remains dynamic-QP rejected. No candidate is accepted;
these combined representations must not be promoted on the numerical solve alone.

R459's native launch witness driver failed compilation because its adapter call
omitted the required physical tolerance argument. R460 separately corrects that
setup and tests a finite set of same-model steering-preparation/forward sequences.
These are physical feasibility oracles, not unchanged-affine-QP solutions or live
normal authority. Original bounds, map/peer and terminal proofs remain required.

The first metadata seal used an incorrect R448 row key; it wrote the payload
manifest but did not write the registry. A separate metadata resume completed the
registry. A host SHA verifier initially called a Python3.11 API on Python3.10;
streamed SHA256 then verified all1358 original payloads. Both setup failures are
preserved separately from runtime/controller evidence.

R460 completes the native physical oracle. Four negative-steering candidates with
2/4/6/8 preparation stages pass the complete unchanged exact wall, dynamic and
terminal-successor proofs. At2stages (0.1799104612s) the1.796537451s trajectory ends
at1.005506635m/s, lag0.772056763m, ey0.891378012m. Virtual progress is held0; physical
motion is represented by lag and must not be reported as virtual progress. With
no preparation, the same negative steering/forward input contacts1wallcell;
positive-steering candidates reject either the lateral corridor or exact wall.
This proves a forward physical witness exists for this source. It does not solve
the original affine QP, demonstrate timing/receiver authority or authorize a
fixed steering/acceleration normal fallback. Next test the witness as a same-model
candidate/linearization seed with a constrained rest-preparation segment and
otherwise optimized controls, retaining all normal proofs.

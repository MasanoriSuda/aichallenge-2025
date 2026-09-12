# r76 current wall proof and unavailable replacement sources

2026-09-12 JST. Production baseline/rollback `56692749`; build116 all26packages,
package103 all2598records passed before this run. No production control change
in this audit. Authorized M4–M6 work continues; race acceptance remains open.

Standard dev2-r76 automatically ended at D1decision3999, actual0.60m/s, wp31,
after both cars spent most of the initial90seconds stationary. No laps.
The proposed manual-stop helper found no runner and failed before any write or
signal: automatic failure teardown had already finished. There was no manual
termination or changed failure criterion. Original DLL and user JSON were restored.

## Current proof: same source, history, domain and physical failure

The scheduled-admission-active snapshot3999 contains the exact current observation,
original request and transactions. Source3454/job3993/index3, geometry7297210173200458920.
Prior indices0–2 published; index3 correctly fails closed. Current context/prefix
are compatible; measurement5 is PoseMismatch, causing an independent current
physical proof. Current5/world4/physical6 denotes a wall rejection at98.619997805.
This is distinct from r75 D2Start/update_v_max and earlier publication deadlines.

R475 rebuilds the replay against build116. It verifies bit-exact retained history,
original source input fingerprint12129506801317380018 and regenerated domain
fingerprint11635038331997886933. All three replays reproduce the same failure time
and reason. Current proof timing3.606–3.861ms is unloaded offline evidence only;
recorded live time was9.064267ms wall/6.248292ms CPU. The original source certifies.

R477 instruments the unchanged checker: failed occupied cell row631/column189,
center(89627.593870164274,43129.85165326744). Current compiled calculation rejects
at98.619997805; native calculation without compiled maps rejects at98.624997805,
one existing5ms step later. Both retain the same inputs, original hard footprint,
250ms receiver range and current-world rules. Compiled calculations alone are
not the cause. R476's cell-index diagnostic compile failure remains preserved.

R478 checks all already-existing branch corner sweeps separately while retaining
the original full body ranges, branches and intervals. Both variants still reject
at their original times. Added417/423 tile checks provide no accepted candidate.
No branch-count, tolerance, epsilon, receiver age or publication window is changed.

R479 uses four native point samples from the original per-step input abstraction.
Earliest-braking samples remain clear. Latest-braking samples hit the same cell
at98.644997805/98.634997805 for lower/upper steering. This is the footprint with
original0.2m lateral clearance and0.05m margin, not a claim of an observed simulator
body collision. The samples do not prove a feasible actual receiver trace or any
robust safety certificate. They show that removing numerical hull overestimation
alone cannot certify the complete current input abstraction.
[Generated plot](../../../output/20260912-nine-state-r76-wall-figure/wall-and-braking.png).

## Replacement-source failures and architecture comparison

D1source3488/decision3993/time98.009997809, geometry7297210173200458920, failed its
wall-refinement QP just before the current-proof failure. Its exact578-row/249-variable
strict affine problem is infeasible in R481; phase-I uniform slack0.0013818656731462914.
This concerns exact affine constraints, not nonlinear reachability or accepted
solver-residual tolerances. R480 all nine A/B/C/D/G arms reject this moving scene.

Separate stationary D1source2025/decision2529/time57.679998710 has geometry
10749198986430161601. Its saved strict affine rows are feasible with maximum
residual4.85722573273506e-16. R480 rough-left C and offline-left D each find a
fully certified source after20candidate attempts; A/B/right/G reject. Thus physical
feasibility exists for this stationary world, but no live or bounded production
candidate has been promoted. It is not the r72 advancing-Store failure world.

R482 eight ablations per source isolate pose-specific future wall rows, independent
solver equilibration and a connected native steering-away tangent. Original control
variables remain free; state/world/model/margins stay fixed. All16 reject. The four
moving pose-wall arms stop in prototype corridor construction, so their rejection
cannot be read as a physical impossibility proof. R483 identifies the first prototype boundary: future stage5 (zero-based4),
ey0.904959 within[-3.711395,1.000245], theta0.126176, lag0.032065, heading0.097385;
the provisional footprint is not clear. Its diagnostic exception then crosses a
noexcept boundary and aborts; only the first arm is observed, no later arms or
completed comparison are claimed. A corridor builder must be able to choose
a safe interval before the provisional QP trajectory itself has been certified. Current/initial map metadata is not
changed again, and no old static donor is substituted into these new snapshots.

## Run coverage and next work

Whole-run command source Hz D1/D2=35.6244/35.6095, maximum gap55ms both;
no duplicate/backward stamps or nonfinite commands. Callback maxima25.870022/
27.039479ms, one over25ms each, no adjacent pairs. Whole-run metrics include
startup and teardown, and publication is not application acknowledgement.

R484 replaces only that prototype query with the existing complete lateral-run
API at the actual future lag/heading, original scalar bounds, hard footprint and
boundary guard. Corridor generation proceeds, but all8arms still reject downstream
wall/dynamic/numerical problems. This provisional correction is not promoted.

The follow-up R485–R493 audit separates candidate topology from world feasibility.
Saved moving3488 is one derived candidate with forced pass stage18 and ahead20;
its strict affine infeasibility is not a physical impossibility certificate. R486
finds6 complete native physical witnesses without requiring that derived schedule.
R488/R490 explicitly reset and reseal the candidate schedule. With the original
production wall, R490 certifies a moving free-input candidate and two stationary
variants. Seeds and topology changes remain diagnostic. R487 full-pose wall rows
are unpromoted; R489 compile failure is retained; prepared R491 was not executed.

R492 exact QPs isolate native nine-state conditioning, and R493 preserves already
certified current Follow/Cruise sources under both policies. The bounded production
slice is the [native numerical owner](native-nine-state-numerical-owner.md), with
[follow-up evidence](native-nine-state-numerical-evidence.json). Original current3999
must remain rejected. Build, current replay and standard live acceptance follow.
All M4–M6, repeated runs/gates and same-artifact submission/eval remain open.

[Sealed payloads](r76-evidence.json). All diagnostic candidates have no live authority.

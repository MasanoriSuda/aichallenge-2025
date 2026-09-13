# Recovery direction failure observation

2026-09-13. Baseline89004222, runtime6f42d134/single-r21. Authorized autonomous
implementation/measurement/local commits, no push. M1–M6 remain incomplete.
[Boundary evidence](rejoin-preparation-live-audit.md).

The earliest unresolved direction boundary is the producer inside
`evaluate_recovery_safety`: the supervisor enters CheckClearance
but receives Unknown and stops. Aggregated course-rejection/contact flags cannot
identify each actual physical candidate. The downstream stop cancels normal
generation; that cancellation does not establish an async admission defect.
No candidate infeasibility or detector false positive is established yet.

Observe one first failed CheckClearance direction selection per process. Preserve
the original map cells/geometry/axis, footprint, pose, lateral/heading state, exact
primitive/rate-independent kinematic rollout parameters and contact policy for
each actually evaluated trial, native result summary and endpoint. Record the
original course-guard request and its diagnostic outcome separately from physical
feasibility. Include evaluation permissions, decision/ROS/steady clocks, selected
direction and final static/V2X completeness. A finite diagnostic capacity must mark
overflow explicitly; overflow is never complete evidence or a control rejection.

Trial recording must not add/reorder/prune candidates or change selection, safety,
detector, gear, retry, model, bounds, margins, clocks or normal authority. Collect
compact summaries only while CheckClearance can consume full rollouts. Retain the
map via const shared ownership after its existing one-time construction; no grid
copy/hash/I/O in the control callback. One owned asynchronous write drains at node
shutdown. A failed diagnostic remains a failed observation, with no authority.
Use a separate internal schema and output directory; never consume normal failure
buckets or overwrite an existing run.

Verify that serialization preserves replayable original physical/course positives
and negatives, bounded overflow and atomic duplicate handling. Run source checks,
focused native tests, build137/package124, review/seal/local commit; then original-DLL
single-r22 to obtain the new observation. Reproduce each recorded trial through
the unchanged native functions before deciding a production repair. If this world
does not reach the boundary, keep the observation inconclusive and obtain another
explicitly justified dimension. The future native Rejoin comparison is separate
from the kinematic Recovery candidate oracle; no raw primitive becomes normal.

Rollback is the observation slice after89004222. Remove or promote this diagnostic
only with an identified direction producer repair and preserved regression input.
Physical observation truth, current/retained speed, missing later wall geometry,
r80D2 actual-send timing and remaining integrated M4–M6 stay open.

R760 all245native (Recovery151, footprint65, snapshot29) and source118pass.
The new saved-map replay separates physical positive/course negative, physical
negative and accepted reverse; capacity overflow stays incomplete, duplicates
preserve the first file. Build137all26/source unchanged, Summary: 2647 tests, 0 errors, 0 failures, 0 skipped.
The writer runs in a node-owned future; no map hash/copy/I/O occurs in control.
V2X is summary only and course_preview is not selection. No actual failed
direction record yet; fixed single-r22 and exact per-trial replay follow.
[Validation/review](recovery-direction-observation-validation.json).
[Internal schema](../../../docs/interface/mpcc-recovery-direction-observation.md).

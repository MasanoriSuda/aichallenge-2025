# First actual rejected interval, independent of prior normal movement

2026-09-13, baseline176bb53d/controller38a562ee. R811 seals single-r27:
the first cached current-interval rejection appears at decision1843, before the
first normal send2063 and authenticated moving witness2534. This falsifies the
observation's assumption that the relevant query follows normal movement.
Cached last_physical text does not identify a new query on subsequent callbacks.
The actual live get_control calls init_problem; a worker-ownership defect is not
established. No wall algorithm defect or physical infeasibility is established.

Replace the post-motion-only eligibility with the first rejected original live
ordinary query. Preserve the worker/behavior-override exclusions, one attempt,
fixed32 run records and owned asynchronous writer with immutable shared grid.
The predecessor is optional evidence, never a manufactured normal send. Use a
new internal v2 schema with prior_moving null or a complete validated mapping;
old v1 remains historical and its required flat predecessor semantics are unchanged.
Original geometry, selected interval, safety predicate and control remain identical.
No extra query, map copy, hash or filesystem operation on the control callback.

Before/after regression: a rejected preferred-position query with no earlier normal
send cannot be written by the old recorder, and is faithfully saved by the new
recorder. Existing original-grid replay, overflow, duplicate and invalid-bound
tests remain. Also reject inconsistent, nonfinite or nonpreceding witness metadata.
Verify native/source tests, full build/package, and fixed-commit single-r28 with
standard rendering, original physics/DLL, six-lap configuration and120hostsec
observation bound. Stop on actual publication failure. Exact original query replay
precedes any producer repair. No renderer comparison or race acceptance from this run.

Added authority: none. Removed path: post-moving prerequisite for this diagnostic
only. Rollback: eventual observation-scope commit. Review deletion after single-r28
if absent; retire after a supported producer regression/fix if captured. This is
a detection-gap repair, not another physical solver repair. Deadline failures and
all outstanding M4–M6 acceptance remain open.

R812 reproduces one old failure (other32pass); R813 native34/source118 pass.
Build141 all26packages; package128 all2660tests, zero errors/failures/skips.
Original controller text matches after removing only the observer block.
Fixed-commit single-r28 next. [Validation](first-wall-interval-validation.json).

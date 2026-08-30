# Design

## Root cause

`build_rate_resolved_track_cruise_physical_snapshot()` currently derives its
course-frame knot window only from the ego progress state bounds.  The same
knots are later consumed by `rebuild_target_horizon()` to project the selected
peer's current-world Cartesian prediction.  A peer may legitimately be ahead
of the ego's finite reachability horizon, so both left and right candidates
lose their target projection even though the peer and course are valid.

The failure is not physical infeasibility.  It is a provenance-domain mismatch:
an ego wall-proof window is being reused as an interaction projection window.

## Initial hypothesis and refutation

The first implementation expanded the frame window to the union of ego and
target support.  It passed all 55 tests, but
`output/20260830-111337/d1/autoware.log` reproduced the exact same stage-zero
failure six times around waypoint 338.  The production change was therefore
removed instead of being retained as an unproven patch.

## Diagnostic change

The projector currently collapses several distinct failures into one invalid
boolean.  It also returns immediately when any knot pair is non-finite,
non-monotone, or geometrically degenerate.  Record the exact category, segment,
target point, monotone lower bound and knot window in the existing immutable
worker rejection.

The diagnostic step did not change acceptance.  Only after the static and
dynamic evidence below identified the duplicated circular seam did this Slice
change projection behavior for zero-length segments.

## Confirmed root cause

The runtime failure occurs around waypoint 338.  In
`env/final_ver3/traj_mincurv_org.csv`, waypoint 348 and waypoint 0 have the
same `(x, y)` coordinate because the CSV explicitly closes the circular path.
The forward horizon crosses the sequence `... -> 348 -> 0 -> ...`, so the
recorded course contains one zero-length geometric segment with monotonically
increasing unwrapped progress.

`project_to_recorded_course()` treated any zero-length segment as corruption
of the entire course.  That invalidated both homotopies before the solver even
though adjacent segments remained valid.

## Root correction

Ignore zero-length segments when finding the nearest geometric projection.
They contain no projection surface.  Continue to reject non-finite knots,
non-monotone progress, and a window in which no usable segment exists.  This
preserves fail-closed behavior without allowing an irrelevant circular seam to
erase valid current-world candidates.

Candidate geometry, solver configuration, certificates, authority and runtime
thresholds remain unchanged.

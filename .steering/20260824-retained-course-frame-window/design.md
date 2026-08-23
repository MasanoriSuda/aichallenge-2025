# Design

## Observed symptom

`output/20260824-043223` produced physically certified Overtake canonical plans,
but current-world revalidation rejected 23 cycles as
`course-frame-unavailable`. The same outcome also appears in Follow production
and briefly forces the emergency owner.

## Root-cause chain

1. Async solving and plant/model differences make the retained plan's expected
   current progress differ slightly from newly measured progress.
2. `lift_progress_to_retained_branch` correctly accepts that difference within
   the continuity tolerance.
3. `build_progress_course_frame_knots` anchors its first knot at the newly
   measured progress and only extends forward.
4. When the retained expected current state is behind that first knot,
   `sample_course_frame` fails closed.
5. A continuity-valid plan is therefore rejected for missing geometry that the
   current reference path can provide.

The problem is not a permissive/strict clearance parameter. It is an asymmetric
course-frame provenance window.

## Repair

- Define one retained course-frame progress range from:
  - lifted measured origin;
  - retained expected current progress;
  - every remaining retained endpoint.
- Extend the current reference-path course-frame knots both backward and
  forward to cover that closed interval.
- Fingerprint the resulting exact window and continue using the existing wall,
  corridor, target, and identity proofs unchanged.

## Why this is not a fallback

No rejected plan is accepted without proof. The repair only supplies missing
current reference geometry for states already admitted by the existing progress
continuity gate. All reconstructed paths still undergo swept footprint, wall,
and dynamic corridor validation.

## Remaining hypothesis

`stage-corridor-violation` may be either a valid rejection of a moving corridor
or a time/progress alignment defect. It is deliberately not relaxed in this
Slice. Dynamic evidence after the course-frame repair determines the next step.

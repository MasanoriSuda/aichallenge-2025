# Design

Extend the typed Overtake retained-world result with an immutable time-zero
corridor diagnostic:

- measured and expected lateral position;
- corridor lower and upper bound;
- signed measured and expected minimum reserve;
- measured and expected current progress;
- first retained control-stage index.

Populate it inside `build_overtake_current_world_retained_proof()` from the
same request, cursor, window and corridor sample used by admission. Propagate it
through the canonical evaluation result and include it in the existing
throttled canonical telemetry and failure detail.

No value is used for control, admission or phase changes. This Slice is
observation-only and ends with a dynamic root-cause classification, not a
performance change.

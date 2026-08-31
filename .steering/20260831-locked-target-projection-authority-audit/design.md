# Design

## Hypotheses

1. **Raw observation loss**: `/v2x/vehicle_positions` stopped carrying d2.
2. **Course projection seam**: d2 remained physically observed, but bounded
   common-course projection failed near the hairpin and `locked_seen` inherited
   that failure.
3. **Provenance mismatch**: the observation was valid, but generation or
   target identity changed before target join.
4. **Genuine infeasibility**: d2 was outside both course and bounded local
   geometry and the last artifact no longer had a current dynamic proof.

## Audit flow

`V2X message -> raw d2 pose -> ego-local geometry -> common-course projection
-> locked-target continuity -> target provenance -> seven-state certificate
-> phase transition`

The repair, if justified, must not retain an old path merely because a Mission
exists. It must distinguish an observed target whose course coordinate is
temporarily unresolved from an actually missing/stale target.

## Frozen evidence

The MCAP contains `d2` every approximately 50 ms throughout the terminal
window.  At the first Behavior drop its Euclidean distance is approximately
14 m and its ego-frame lateral displacement is approximately 13--14 m, so the
bounded 6 m near-field fallback correctly does not claim a collision geometry.

The same target projects to alternating forward course distances of
approximately 23.7 m and 26.7 m while the configured *entry/front detection*
window is 24.0 m.  The projection therefore alternates valid/invalid at a
hairpin even though target identity, V2X generation, receipt time and raw pose
remain current.  `locked_target_seen` currently inherits that bounded
projection result.  After `target_hold_sec`, `resolve_target_continuity()`
labels the condition `locked target stale or lost` and mutates ShiftOut to
Recovery.  At the same epoch the canonical publisher still has a current-world
certified ShiftOut Bundle with more than 12 m dynamic clearance.

This disproves raw observation loss and physical infeasibility.  It is a
classification/lifecycle defect: one boolean represents both (a) raw identity
observation and (b) membership in the bounded tactical detection horizon.

## Repair alternatives

1. Increase 24 m, the 6 m fallback or the hold time. Rejected: this only moves
   the same seam and violates the frozen-Slice constraints.
2. Treat any raw Cartesian observation as a course-relative obstacle. Rejected:
   a hairpin may place another track branch nearby and the resulting longitudinal
   relation is not certified.
3. Keep the bounded projection for tactical entry/front classification, but
   give an already locked ShiftOut/Pass target a separate continuity projection.
   This projection is permitted only when a previous course-progress identity
   exists, retains the existing physical progress-change constraint and scans
   at most one reference-path lap. Selected.

The selected repair does not widen the tactical detection window.  A target at
26.7 m remains outside front/entry selection because all such decisions retain
the original 24 m bound.  It only prevents that tactical fact from being
misreported as transport loss.  Position jump and course-progress discontinuity
remain hard rejection.  The current-world seven-state producer continues to
own whether a ShiftOut/Pass artifact is executable.

## Causal statement

The bounded entry/front projection was reused as the locked-target identity
continuity source.  At a hairpin, a continuously observed d2 crossed that
bounded projection seam, which stopped refreshing target continuity; the
persistent phase lifecycle then emitted a false terminal Recovery while an
independently certified ShiftOut artifact was still publishable.  Separating
locked-target continuity projection from tactical detection removes the
upstream semantic conflation instead of adding another terminal exception.

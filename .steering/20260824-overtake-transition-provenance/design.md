# Design

## Root-cause chain

1. `replan_early_shiftout_side()` can replace the active side without
   incrementing `mission_generation`.
2. `MpccProblemContext`, async `ContextLifecycleState`, and retained execution
   provenance do not name the selected side.
3. A worker/store artifact from the previous homotopy therefore remains
   semantically indistinguishable from the new homotopy until its lateral path
   is checked against the new corridor.
4. The evaluator then reuses one mutable result for the incoming and stored
   artifacts. A stored-plan failure can overwrite the incoming failure, and a
   successful retained selection is logged as `fresh-selected` merely because
   the aggregate result is complete.
5. Transition-local corridor/progress counters consequently mix valid current
   corridor rejection, stale old-side rejection, and fresh-plan rejection.

This is an identity and artifact-lifecycle defect, not evidence that the
corridor clearance should be relaxed.

## Repair

### Exact homotopy identity

Add `execution_side_sign` to the canonical problem context and its fingerprint.
For ShiftOut, Pass and Return it must be -1 or +1. For Track, Cruise and Follow
it must be 0. Carry the same value through current retained provenance.

### Lifecycle boundary

Add the side to the async semantic context. A side change advances the epoch,
resets the latest-only mailbox, clears the accepted plan from the old family,
and causes the dedicated solver context to cold-reset through its existing
side identity. Observation updates on the same side continue to share the
epoch.

### Fail early

Check retained plan intent, Mission generation, target and side before building
course-frame, wall or dynamic-corridor evidence. A semantic mismatch must not
be misreported as a physical corridor failure.

### Source-separated evidence

Evaluate the incoming worker plan and stored plan into independent results.
Record:

- incoming attempt/outcome/plan ID;
- stored attempt/outcome/plan ID;
- selected source: none, incoming, or stored.

Only the selected result is copied into the command candidate. A failed stored
evaluation cannot overwrite an incoming diagnostic.

## Non-goals

- No corridor relaxation.
- No side-selection strategy change.
- No additional fallback or retry.
- No production authority promotion.

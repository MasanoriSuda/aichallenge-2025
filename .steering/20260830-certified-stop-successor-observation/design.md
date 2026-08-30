# Design

## Root cause being tested

The current certificate and publisher have different owners:

1. retained revalidation proves one normal publisher interval followed by a
   maximum-braking, path-tracking Stop suffix;
2. only a boolean and state trajectory reach the production adapter;
3. after normal authority disappears, `canonical_normal_emergency_stop()`
   regenerates a different base-track Stop command;
4. the certified suffix is never joined to the next observed control state and
   is never shown to be executable by the publisher.

This slice does not assume that executing the old suffix is correct.  It first
tests the missing causal join.

## Immutable observation artifact

Extend terminal Stop evidence with a sample-aligned actuation trace:

- elapsed time and duration;
- acceleration and steering rate applied during the sample;
- end velocity and end steering;
- number of samples belonging to the already-published normal interval.

The exact physical trajectory remains the pose/state authority.  The new trace
only preserves the inputs which generated those exact states.

Production authority carries the evidence as diagnostic data.  After the final
canonical command matches the serialized command, the evidence is promoted to
`PublishedCertifiedStopSuccessorObservation` with its source decision,
solution, fingerprint, intent, and control-origin time.

## Join

At the next current-world normal evaluation:

- compute elapsed time from the source control origin to the new control
  origin;
- sample the old exact Stop successor at that elapsed time;
- reconstruct its expected world pose using the sealed physical snapshot;
- compare it with the new predicted control-origin pose, speed, and steering;
- emit a single structured log and retire the observation.

No tolerance grants authority.  Tolerances only classify whether the old
certificate and the real publisher share a usable causal boundary.

## Follow-up classification

- Join succeeds and later generic Stop fails: certificate/publisher ownership
  mismatch is confirmed; a later slice may execute the certified successor.
- Join fails at state or actuation: model/publisher mismatch must be fixed
  before any successor authority is wired.
- Join fails by time/identity: scheduling or lifecycle provenance is wrong.


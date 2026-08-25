# Design

## Before

```text
Track/Cruise/ShiftOut/Pass/Return
  -> shared rate-resolved six-state worker/store/publisher

Follow
  -> dedicated five-state worker/store/transition admission/publisher
```

Changing from Cruise to Follow or Follow to ShiftOut changes both intent and
mathematical formulation.  The later ShiftOut atomic admission must then bridge
from an unrelated previous owner.

## After

```text
Track/Cruise/Follow/ShiftOut/Pass/Return
  -> one rate-resolved submission draft
  -> one six-state solver/artifact
  -> exact physical proof
  -> one retained current-world proof
  -> one normal publisher
```

Intent remains part of the immutable source context.  An intent transition
cannot consume the preceding intent's artifact; it synchronously creates the
first same-formulation artifact, stores it, then requires the ordinary
current-world join before publication.

## Follow semantics

The existing `FollowLongitudinalContract` remains the single source of:

- target progress horizon;
- ego progress lower/upper bounds;
- desired planning gap and hard gap;
- velocity reference and hard upper bound.

`build_extended_progress_problem()` already installs this contract in the
five-state semantic state/input rows and explicit gap constraints.  The
rate-resolved adapter preserves the state bounds and longitudinal references
while replacing curvature input with steering state/rate input.  No Follow
parameter is reinterpreted or duplicated.

Fresh and retained request assembly use the same typed normal-scope resolver.
Track/Cruise metadata, Follow metadata, and Overtake execution metadata are
capabilities; the resolver maps exactly one capability to the selected intent.
This prevents an intent from being admitted by the submission boundary while
the semantic request builder silently omits it.

For retained Follow execution, physical dynamic-footprint revalidation is not
a replacement for the longitudinal hard-gap contract.  The shared six-state
retained proof therefore joins the current target identity/generation and
checks the current gap plus every remaining stage using effective ego progress
`course_origin + progress + lag`.  A missing, stale, mismatched, or hard-gap
violating target fails closed as a typed reason.

## Deletion boundary

After routing Follow through the shared owner, delete only Follow-specific old
normal infrastructure.  Shared tactical five-state planning used to construct
semantic requests is not command authority and remains until separately
audited.  Rejoin remains outside this Slice.

# Requirements

## Objective

Turn one accepted six-state steering-rate solve and its exact physical wall
proof into one immutable, atomically retained certified plan.  This is the
next prerequisite for current-world retained admission; it does not grant
production authority.

## Root cause

The serialized worker currently publishes the numerical result and physical
wall result through independent mailboxes.  Although both carry provenance,
there is no object whose validity means "this exact artifact passed this exact
physical proof", and there is no monotonic store for that object.  A later
retained path would otherwise have to reconstruct the join from independently
consumed latest results, which permits mismatched or missing evidence.

## Invariants

- Only an `Accepted` physical result may produce a certified plan.
- Artifact and physical identities must match in every field.
- Invalid or stale replacement must preserve the last accepted plan.
- The store owns an immutable artifact through shared lifetime.
- No age-only admission, current-world revalidation, command publication,
  fallback, solver setting or parameter change is included in this Slice.
- Runtime remains `authority=shadow, selected=0`.

## Definition of done

- Pure deterministic tests cover accepted, rejected, mismatched, stale and
  failed replacement paths.
- The serialized worker publishes a certified plan only after exact physical
  acceptance.
- Runtime telemetry distinguishes certified-plan store acceptance/rejection.
- Existing authority source contract proves that the new store cannot publish
  a command.
- Full build and package tests pass.

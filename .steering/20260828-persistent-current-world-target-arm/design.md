# Design: one-variable architecture comparison

## Controlled comparison

```text
persistent-a
  captured Mission geometry
  captured target binding

persistent-target-bound-a2
  captured Mission geometry
  current-world target binding

stateless-left/right-b
  rebuilt reference geometry
  current-world target binding
```

Only the target binding differs between A and A2.  Only the geometry differs
between A2 and the same-side B arm.  This prevents a successful B result from
being misclassified as proof that the whole Mission lifecycle must be removed.

## Construction

The A2 builder copies the sealed source snapshot, validates the selected side,
rebuilds target stages through `rebuild_target_horizon()`, activates dynamic
obstacle refinement and replaces only the target stages.  It then recomputes
the interaction fingerprint.  References, bounds, wall inputs, request timing
and terminal-successor policy remain untouched.

The result remains audit-only: it has no command conversion, store, mailbox or
publisher.

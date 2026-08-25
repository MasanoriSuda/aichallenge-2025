# Design

## Reachability graph

```text
live tactical branch worker
  -> five-state left/right solve
  -> immutable pre-entry identity + physical proof
  -> Mission Gate A only
  -> six-state transition admission
  -> six-state normal publication
```

The above remains temporarily live. It is not a second command owner.

The removable graph is:

```text
evaluate_overtake_async_shadow()       # no callers
  -> submit five-state async worker
  -> retained current-world selector
  -> five-state CanonicalNormalSelection
  -> canonical_normal_control()        # no callers
```

Keeping this graph makes the retired owner reconnectable and obscures which
solver actually owns actuation. It provides no runtime fallback because its
root has no call site.

## Selected change

1. Add source contracts before deletion.
2. Delete `canonical_normal_control()`.
3. Delete the no-caller Overtake async/retained selector, its mailbox,
   telemetry and retained plan-store state.
4. Narrow the remaining lifecycle to the tactical five-state solver warm
   state needed by left/right pre-entry evaluation.
5. Replace Emergency's fabricated five-state context with `Unresolved`.
6. Preserve the existing pure pre-entry resolver and its no-actuation
   contract.

## Follow-up boundary

The next Slice must create a prospective six-state pre-entry artifact and
promote it in the same change that deletes the live tactical five-state Gate
A. It may not merely drop Gate A and let Mission state mutate before evidence
exists.

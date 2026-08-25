# Design

## Authority boundary

```text
selected tactical homotopy
  -> causal selected-side six-state solve outside callback
  -> exact wall/target proof
  -> current-world join at live Gate A
  -> atomic Mission + CertifiedPlan proposal
  -> freeze Mission geometry
  -> transition Idle/FollowPrepare -> ShiftOut
  -> existing six-state intent-transition admission
  -> six-state normal command
```

The proposal is an admission certificate, not a second command owner. The
post-transition production solve remains mandatory because it owns the actual
new-intent command and certified-plan store.

## Fail-closed rules

Fresh ShiftOut stays on Follow/Cruise when any of these differ:

- target id;
- selected side;
- prospective Mission generation;
- proposal intent (must be exactly `ShiftOut`);
- Mission side and CertifiedPlan execution side;
- current tactical/context identity.

No proposal is retained across cycles and no grace period is introduced.

Target provenance is validated exactly once when the worker result is joined
to the live world. The proposal then seals the worker artifact's source target
generation. The FSM verifies the sealed proposal against the CertifiedPlan; it
must not reinterpret that source generation as if it were the latest V2X
generation. A newer observation generation is valid only when the existing
target-continuity validator has accepted it.

## Direct Pass

Direct Pass keeps its existing five-state entry proof in this Slice. Mixing it
with ShiftOut would infer dynamic evidence which has not been observed. Its
separate promotion Slice must first produce and observe an exact Pass proposal,
then delete the remaining five-state Gate-A entry path atomically.

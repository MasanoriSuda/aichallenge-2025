# Design: make executed intent an upstream problem identity

## Root cause

`rate_resolved_normal_production_control()` resolves proposed versus previous
intent only after `init_problem()` and submission-draft construction. This is
late enough to retain the old command, but too late to produce its successor.

```text
tactical proposal Follow
  -> init_problem(Follow)
  -> build submission draft(Follow)
  -> current-world proof retains previous ShiftOut
  -> publish ShiftOut
  -> enqueue Follow successor
  -> ShiftOut artifact eventually exhausts
  -> no normal authority -> Emergency
```

The publisher is therefore not the right place to repair continuity. The
executed identity must enter before authority resolution and semantic problem
assembly.

## Resolution

1. Read the last actually executed certified plan from the monotonic store.
2. Accept it as a retained execution identity only if:
   - the plan and artifact validate;
   - its intent equals the last successfully published intent;
   - the intent is ShiftOut, Pass, or Return;
   - target, generation and side are complete;
   - the artifact cursor is available at the same predicted control origin
     used by new submissions.
3. Pass that typed identity into `resolve_canonical_execution_identity()`.
4. Preserve priority: live OvertakeLine, then live DynamicEscape, then retained
   executed artifact.
5. Derive `AuthorityRequest.phase`, problem intent and execution metadata from
   the resolved identity.
6. Build and enqueue the next asynchronous QP for that effective identity.

## Lifetime and safety

This is not a new fallback. The retained artifact is already the command owner
after current-world revalidation. The change makes problem generation agree
with that owner.

The cursor is the sole lifetime proof. When it is future, exhausted or
invalid, the retained request is inactive. Current wall and dynamic-obstacle
proofs remain mandatory for command publication and for the newly solved
successor.

## Alternatives rejected

- Extend horizon/cursor: falsified dynamically and only delays the hole.
- Add a grace timeout or Mission lease: creates authority without executable
  evidence.
- Relabel a Follow QP as ShiftOut: violates problem fingerprint semantics.
- Rebuild synchronously after late atomic resolution: duplicates the solver in
  the real-time callback and preserves split-brain problem ownership.
- Retain the old command indefinitely: violates current-world and cursor proof.

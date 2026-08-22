# Design

## Causal chain

```text
five-state QP solves e_psi
  -> legacy conversion keeps e_y/e_psi/progress but later extraction retains only e_y/progress
  -> wall certificate reconstructs heading from delta(e_y)/delta(s)
  -> certified pose can differ from the QP state pose
  -> a footprint-safe bound refinement would be based on the wrong orientation
```

The legacy reconstruction remains required by existing production consumers during migration. The
new contract is additive and shadow-only.

## Contract

`ExtendedExecutionTrajectory` stores stage 1..N:

- effective path distance;
- lateral state;
- heading-error state;
- velocity state;
- absolute progress;
- minimum lateral QP-bound reserve.

Extraction validates dimensions, finite state, increasing path distance, monotonic progress and the
same applied lateral bounds as the QP. It does not reinterpret heading from the lateral profile.

`AlignedMpccExecutionTrajectory` may carry exact heading offsets. The physical validator uses them
when the vector has horizon size, otherwise it preserves the legacy slope-derived path.

## Decision after replay

- If contact rejects disappear, the earlier certificate used an inconsistent orientation.
- If positive-reserve contact remains, physical map/footprint bounds must be inserted into the QP.
- If swept-only rejects remain at index 1, current-pose-to-stage-0 reachability needs a separate
  first-stage constraint or seed.

No authority promotion is permitted in this slice.

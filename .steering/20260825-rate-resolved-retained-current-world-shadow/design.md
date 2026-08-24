# Design

## Causal chain

1. A six-state solve and its full physical proof are accepted.
2. The store serializes only artifact plus result.
3. Static-world provenance used by the proof is discarded.
4. A later retained consumer can resolve time, but cannot establish that the
   old wall proof belongs to its current world or join safely from its current
   state.
5. Consequently production promotion would either accept on age alone or add
   ad-hoc downstream guards.

The fix is to preserve the proof source and add one explicit current-world
admission object before any authority promotion.

## Certified source evidence

`CertifiedPlan` additionally owns the immutable wall-grid shared owner,
footprint, course-frame knots and physical sampling policy from the exact
accepted snapshot.  The builder performs an all-or-nothing three-way join:

`ExecutionArtifact identity == Physical Snapshot identity == Physical Result identity`

## Retained current-world proof

The shadow evaluator performs, in order:

1. validate certified plan and resolve its time cursor;
2. match current Track/Cruise intent;
3. require a current empty dynamic-obstacle observation;
4. match static wall owner, footprint and sealed course-frame fingerprint;
5. interpolate the expected retained state at the cursor;
6. lift measured progress onto the retained circular branch;
7. extract rate-resolved actuation and prove steering/speed reachability;
8. sweep the measured-to-control prefix and connector to the expected state.

The already accepted suffix is not resampled.  It is immutable and remains
bound to the same wall-grid and course-frame evidence.  Resampling the entire
suffix in the control callback would duplicate a 1--10 ms physical proof and
reintroduce callback overruns.

## Authority boundary

The output is `RetainedProof`, not `CanonicalNormalCandidate` or a command.
Controller integration only counts and logs outcomes with
`authority=shadow, selected=0`.

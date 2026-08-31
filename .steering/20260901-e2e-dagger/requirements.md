# E2E pre-contact DAgger requirements

## Objective

Relabel states visited by the failed learned student with the admitted LiDAR gap
teacher, without treating the student's commands or physically embedded collision
suffix as expert data.

## Constraints

- Keep the production checkpoint unchanged.
- Reuse the exact runtime preprocessing, NumPy checkpoint and gap-teacher policy.
- Keep only teacher-active correction samples.
- Exclude the suffix beginning one second before a confirmed sub-0.5 m return.
- Record source bag, student checkpoint SHA-256, cutoff and teacher configuration.
- DAgger data is train-only; independent seed 2027 remains validation-only.
- No threshold or runtime policy change is allowed in this slice.

## Definition of Done

1. Collision cutoff and provenance are deterministic and unit-tested.
2. Failed student commands cannot enter the teacher label arrays.
3. Candidate improves the independent corrective-subset metrics.
4. Candidate finishes the single-vehicle gate without stall regression.
5. Candidate passes the seed 2026 runtime NPC gate before promotion.

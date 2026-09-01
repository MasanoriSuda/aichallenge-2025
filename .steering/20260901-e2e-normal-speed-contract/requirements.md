# Requirements

## Objective

Create an immutable normal-state dataset with real speed synchronized to the
same LiDAR timestamps used by corrective teacher data.

## Constraints

- source is `dagger_aggregate_v2`; stored control labels are never reused
- speed comes only from each source bag's `/localization/kinematic_state`
- scans remain physical metres and sequence ordering is preserved
- train/validation split and source identity are immutable
- zero-residual anchors use a distinct schema and label source
- missing bag/topic/type, partial decode or sync violation rejects the sequence

## Definition of Done

- all 3 train and 1 validation source bags derive successfully
- maximum speed sync delta is at most 0.05 s
- loader refuses teacher/direct-policy label sources
- actual speed replaces the earlier diagnostic zero placeholder
- no model or runtime authority changes in this Slice

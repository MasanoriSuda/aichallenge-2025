# Design

Reuse the action-separability probe and append:

- `dagger_aggregate_v2/train` as neutral train sequences;
- `dagger_aggregate_v2/val` as neutral validation only.

The source dataset loader has already normalized scans, so the probe restores
physical metres before the frozen feature extractor.  Sequence IDs are
prefixed with `normal-anchor:` and train/validation overlap is rejected.

The decisive representation is `static_conv5_no_speed`, matching the proposed
LiDAR-only runtime contract.  `temporal_conv5_no_speed` adds only 1/8-step
spatial differences and therefore remains valid for the current scan-only ROS
contract.  Speed variants remain in the report only as controls because the
independent normal corpus does not carry synchronized speed.

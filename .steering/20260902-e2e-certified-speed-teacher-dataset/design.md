# Design

## Sensor synchronization

Move the shared wheel-speed reader into `lib/speed_sync.py`.  A causal matcher
selects the greatest speed timestamp not newer than each LiDAR timestamp:

```text
speed_index = searchsorted(speed_times, scan_time, right) - 1
age = scan_time - speed_time
```

This differs intentionally from nearest-neighbour matching, which can use a
future measurement and cannot reproduce the runtime callback contract.  The
stateful teacher rejects the complete extraction if any admitted scan lacks a
preceding speed sample or exceeds the configured freshness limit.

## Provenance

Add `speed_committed_teacher` as a distinct executed-teacher mode with:

- raw label source `lidar_speed_committed_teacher_dagger`;
- class `LidarSpeedCommittedTeacher`;
- generated control type
  `generated/tiny_lidar_speed_committed_teacher`.

`validate_executed_teacher_certificate` receives the expected mode explicitly.
Its historical default remains `precontact_teacher`, so old callers and old
certificates keep their original meaning.

## Paired labels

For the new teacher, save:

- frozen base steering;
- historical `LidarPrecontactTeacher` steering on the same scan;
- successor steering;
- successor-minus-base runtime target;
- successor-minus-precontact diagnostic upgrade.

The stateful teacher is evaluated once, in timestamp order, using synchronized
wheel speed.  It is never reconstructed independently per accepted sample.

## Boundary

This Slice creates evidence and data only.  Whether the new labels are
representable by the current static spatial adapter is measured in the next
Slice before any training or runtime change.

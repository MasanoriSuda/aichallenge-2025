# Design

## Root cause

The six-state model correctly initializes its steering state from the physical
prediction at `now + delay`.  The immutable execution artifact then integrates
the same physical initial value as though it were the previous desired command.
When actuator lag leaves physical and desired steering apart, extracting a
retained command creates a false desired-command jump.  Live revalidation
correctly rejects that jump, normal authority disappears, and Emergency or
Recovery follows.

The prior publication-horizon Slice corrected which instant was observed and
thereby made this value-meaning substitution visible; it did not create a
second steering predecessor.

## Selected repair

Seal two values with distinct types/fields in every artifact:

- `semantic_initial_steering_rad`: physical state at prediction origin; owns
  vehicle dynamics and wall proof.
- `publication_initial_steering_rad`: last desired command published before the
  solve snapshot; owns desired-command continuity.

Both use the exact certified steering-rate sequence.  Physical trajectory
validation integrates the physical origin.  Command extraction integrates the
publication origin at the artifact cursor elapsed time.  Cursor time already
represents elapsed time since the sealed predecessor snapshot, so it must not
add a second publisher interval.

The QP proves both angle series directly.  For every control stage it appends a
cumulative steering-rate prefix row.  Its lower/upper bounds are the
intersection of the physical-origin and desired-publication-origin angle
limits.  Translating ordinary steering state boxes is insufficient because
those boxes do not constrain the cumulative rate sequence from the second
origin.

The publisher has two exact boundaries.  Before serialization, the solver
command and final actuation remain bit-exact doubles.  After serialization,
the command is compared exactly in the float32 ROS wire representation before
the certified candidate can become the executed retained plan.  This models a
real representation boundary rather than adding a numeric tolerance.

## Rejected alternatives

- Relaxing current-world reachability: hides an unexecutable command.
- Clamping the final steering: creates an uncertified second trajectory.
- Lowering steering-rate/horizon parameters: tuning cannot repair value
  ownership.
- Falling back to five-state MPC: violates single-authority migration.

# Design

## Root cause

The overtake horizon and solved MPCC trajectory use different vehicle yaw for
static-wall footprint sampling. A lateral profile with a steep `d(s)` slope is
therefore optimistic during planning and can lose execution authority later.

## Changes

1. Add a heading-aware lateral-clearance primitive. Lateral translation stays
   in the base reference frame, while footprint yaw includes an explicit path
   heading offset. Keep the existing API as a zero-offset wrapper.
2. Build the nominal overtake lateral profile before per-stage wall repair.
   Derive each stage heading from adjacent `d(s)` samples using the existing
   Frenet heading-reference helper.
3. Use the heading-aware primitive for configured-margin and physical fallback
   clearance searches. This repairs toward the base line only.
4. Revalidate the completed repaired profile with the same heading convention.
5. Use the same heading helper in solved MPCC wall validation so planning and
   execution do not disagree at curved/circular path boundaries.

## Safety and compatibility

- Occupied, unknown and out-of-map cells remain hard failures.
- The physical footprint is never reduced.
- The hard wall margin is unchanged.
- No public ROS interface or configuration schema changes.

# Spatial adapter limited-authority requirements

## Objective

Evaluate the shadow-qualified spatial-plus-speed adapter with a bounded steering
authority while retaining candidate3 as the immutable base and
`fixed_lidar_brake` as longitudinal authority.

## Frozen defaults

- spatial authority is disabled unless explicitly enabled at launch;
- an authority run requires the strict shadow checkpoint and fresh speed;
- missing/stale speed or shadow inference failure falls back to candidate3 for
  that sample;
- legacy residual and spatial authority may not own steering together;
- the shipped production launch remains candidate3 without spatial authority.

## Initial bound

The first A/B limits applied spatial correction to 0.12 rad.  This is below 20%
of the node's 0.64 rad steering limit and bounds the observed 0.358 rad shadow
tail without erasing the normal-state p95 correction (~0.036 rad).  No model,
loss, clearance or longitudinal parameter changes are allowed in this slice.

## Acceptance

- explicit A/B provenance and exact candidate SHA;
- 3/3 laps, zero penalty and zero stall;
- 20 Hz LiDAR handling, no watchdog stale and zero spatial inference error;
- authority is actually applied on nonzero samples but never exceeds 0.12 rad;
- no material lap-time or stability regression against frozen candidate3 runs.

# Design

## Input provenance

Training builders and runtime subscribe to
`autoware_auto_vehicle_msgs/msg/VelocityReport` on
`/vehicle/status/velocity_status`.  Legacy Odometry decoding remains available
only when a dataset builder is invoked with an explicit legacy message type;
it is not the default and is not used by production.

## Artifact identity

The spatial model is a self-contained adapter with an embedded, bit-exact copy
of the admitted base model.  Startup performs two independent checks:

1. the file SHA256 equals the launch contract;
2. all embedded base parameters equal the separately loaded production base.

Failure of either check prevents the controller from starting.  A different
file therefore cannot silently gain steering authority.

## Ownership and rollback

Production uses:

- base TinyLidarNet as the nominal steering producer;
- one bounded spatial adapter as the only learned correction owner;
- LiDAR longitudinal safety as the only acceleration override.

The legacy residual checkpoint remains empty.  Explicitly setting spatial
authority to false is the rollback path and preserves the admitted base
command bit-for-bit.

## Launch boundary

The participant package owns the production defaults because only
`aichallenge_submit/` is guaranteed to be present in the evaluation archive.
Repository-local system launch and environment variables are development
overrides, not production dependencies.

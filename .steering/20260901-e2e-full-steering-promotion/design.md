# Design

The qualified adapter is installed as
`tiny_lidar_net_controller/ckpt/spatial_steering_adapter.npy`.  Participant
launch defaults name this artifact and its SHA directly, so the submitted
archive is self-contained.

`run_autoware.bash` selects the packaged adapter only when both conditions are
true:

1. `AIC_CONTROL_METHOD=tiny_lidar_net`;
2. `TINY_LIDAR_CONTROL_MODE` is empty or `fixed_lidar_brake`.

MPC, `gap_teacher` and `precontact_teacher` therefore keep their previous
authority.  Explicit environment overrides still take precedence.  Setting
`TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED=false` is the rollback and retains the
base output bit-for-bit while leaving optional shadow evidence available.

The spatial candidate owns only steering correction.  LiDAR longitudinal
safety remains the acceleration owner, and the legacy residual checkpoint
remains empty.

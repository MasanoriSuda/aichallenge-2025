# Design

Launch `e2e-single` with only
`TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH` set to the v3 checkpoint.  Leave
`TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED=false`.

Analyze the run with the existing competition and spatial-shadow analyzers.
The experiment intentionally reuses the admitted production controller and
runtime plumbing; it changes only the inspected shadow artifact.

If this gate passes, the next slice may run one limited-authority A/B at the
unchanged `0.12 rad` bound.  If it fails, classify runtime availability,
timing, or normal-distribution leakage before changing the model or bound.

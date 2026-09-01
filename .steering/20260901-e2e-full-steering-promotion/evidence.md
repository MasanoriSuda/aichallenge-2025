# Evidence

## Immutable artifact

The qualified model was copied to
`tiny_lidar_net_controller/ckpt/spatial_steering_adapter.npy` without
conversion.  The source, participant package, installed package and sealed
submission archive all have SHA256:

`f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`

Artifact size is `1,262,771 bytes`.  The archive has one top-level directory,
`aichallenge_submit/`, and contains no Python or pytest cache files.

## Production contract

- participant launch owns the packaged path, expected SHA, base-steering
  conditioning, `1.2 rad` model/authority range and enabled authority;
- unset Docker/Make overrides no longer erase those package defaults;
- custom experimental artifacts are shadow-only unless authority is explicitly
  granted;
- teacher modes force spatial authority off unless an explicit diagnostic
  override is supplied;
- the legacy residual checkpoint remains empty;
- startup rejects a SHA mismatch or model-contract mismatch.

Rollback keeps the packaged model available for shadow diagnostics while
returning the final steering command to the frozen base bit-for-bit:

```bash
make e2e-single TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED=false
```

## Verification

- ML platform: `156 passed`;
- local launch/runtime contract suite: `41 passed`;
- ROS package tests: controller `29`, submit launch `5`, system launch `9`,
  with no failure;
- `make autoware-build`: 25 packages built successfully;
- submission archive layout and embedded model SHA: pass.

The packaged-default run used no ML-workspace path and no spatial environment
override:

- run: `output/20260901-e2e-full-steering-packaged-default-single`;
- laps: `101.069 / 89.156 / 90.400 s`;
- three laps, penalty 0, stall 0;
- spatial authority: 6439/6439 samples, clipping/stale/error 0;
- average inference: `7.12 ms`, minimum scan rate `19.94 Hz`;
- maximum applied correction: `0.52543 rad`.

This proves that the installed and submitted participant defaults—not a
development checkpoint path—own the accepted closed-loop behavior.

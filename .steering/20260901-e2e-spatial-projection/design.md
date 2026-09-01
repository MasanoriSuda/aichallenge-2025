# Design

```text
physical LiDAR
  -> frozen candidate3 conv5 map
  -> immutable Gaussian projection(seed=2026, dim=128)
  -> train-only per-feature fixed standardization
  -> normalized real speed
  -> existing signed continuous correction head
```

The projection matrix is a registered buffer.  It is saved in the candidate
and checked by the strict state loader, but never optimized.

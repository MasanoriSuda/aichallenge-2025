# Design

## Root cause

The admitted gap teacher outputs a braking command below 1.5 m and inhibits
acceleration below 3.0 m. Training intentionally ignores acceleration because the
2026 E2E requirement places ML ownership on lateral control. The deployed `fixed`
student therefore continued at `+0.6 m/s2` even after the same observation that
made the teacher brake, eventually embedding the vehicle between obstacles.

## Authority split

```text
LiDAR -> TinyLidarNet -------------------------> steering
      -> shared frontal-clearance policy -----> acceleration upper authority
```

`fixed_lidar_brake` is a production mode, not a second lateral planner. It cannot
modify the network steering. `gap_teacher` uses the same longitudinal policy and
adds its teacher-only lateral correction.

## Scope

- Extract longitudinal safety calculation from `LidarGapTeacher`.
- Add an explicit decision record and runtime counters.
- Make the safe fixed mode the TinyLidarNet launch default.
- Keep `fixed` available for controlled A/B and backwards diagnostics.

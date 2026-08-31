# Design

## Why this teacher

The current MPCC peer run is not a valid teacher: it loses wall/terminal authority and stalls. Runtime
NPCs do not appear in V2X, so adding another Mission/MPCC patch cannot create trustworthy labels. The
admitted single-vehicle TinyLidarNet already supplies track-following steering. The teacher adds a
bounded Follow-the-Gap residual only when LiDAR indicates a near obstruction.

## Data flow

```text
LaserScan
  -> existing clean/resize/normalize
  -> TinyLidarNet base steering
  -> gap detector over physical (metre) ranges
  -> bounded steering blend + teacher longitudinal caution
  -> /control/command/control_cmd
  -> admitted bag only
  -> TinyLidarNet imitation dataset
```

The gap detector removes an angular safety bubble around the closest forward obstruction, finds the
largest remaining physical opening, and blends toward its center. It does not know vehicle identity or
world coordinates.

## Authority

- `fixed`: production baseline; existing fixed acceleration and ML steering.
- `ai`: existing learned acceleration and ML steering.
- `gap_teacher`: data collection only; base ML steering plus LiDAR gap residual.

The mode is passed as `TINY_LIDAR_CONTROL_MODE` through the existing launch chain. Invalid values or use
with another `control_method` fail before launch.

## Admission

`analyze_e2e_run.py --fail-on-stall` is necessary but not sufficient. A teacher run must also reach
Finish and have no visual/AWSIM collision evidence. Failed teacher output remains `student/other`
evidence and is not extracted.

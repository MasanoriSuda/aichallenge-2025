# Design

## Root cause

The observed contact is not caused by missing outcome metadata or model
capacity.  The historical teacher has two structural omissions:

1. longitudinal thresholds are constant although recoverability scales with
   speed; and
2. gap side is recomputed independently for every scan, so a newly visible
   return can reverse the command after the vehicle no longer has time to cross.

The side-only branch also leaves `+0.6 m/s2` untouched while both side sectors
are closing.  That makes the first frontal return the symptom rather than the
start of the failure.

## Candidate architecture

Keep `LidarPrecontactTeacher` as the immutable geometric proposal generator.
Add a separate `LidarSpeedCommittedTeacher` that owns:

- physical braking and preview envelopes derived from wheel speed;
- an encounter-local side choice with confirmed switching;
- confirmed-clear release;
- bilateral-pinch and opposite-side conflict handling.

The dynamic envelope is derived from configured deceleration rather than an
independent tuning threshold:

```text
stop = static_stop + v * reaction_time + v^2 / (2 * |brake_accel|)
slow = max(static_slow, stop + 0.25 * v)
trigger = max(static_trigger, slow + preview_time * v)
```

Applying this larger trigger to every scan was explicitly rejected by offline
replay: it classified normal curve walls as an encounter and produced 575
side-conflict brakes in 6,479 samples.  The accepted design first evaluates the
immutable historical teacher.  Only when that teacher has already detected a
front/side hazard is the current scan re-evaluated with the speed-dependent
look-ahead.

An opposite-side proposal outside the static slow envelope must persist for two
scans before authority changes.  During the one pending scan, acceleration and
lateral motion are not increased.  A side change first seen inside the static
slow envelope is physically late and commands configured braking plus neutral
steering.  After a side is acquired or confirmed, the published steering is
projected onto the selected candidate sign; the historical severity blend is
not allowed to report one homotopy while still commanding the other.  If both
side sectors are inside the historical side-risk envelope, acceleration is
replaced with braking.

The state retained by this diagnostic teacher is limited to selected side,
pending side/count and consecutive clear observations.  It is reset at the
runtime reset boundary and never enters the production mode.

## Runtime contract

- Mode: `speed_committed_teacher`.
- Speed source: `/vehicle/status/velocity_status`.
- Speed freshness uses the existing wheel-speed freshness duration already
  required by the spatial adapter.
- No fresh speed means no teacher command; the node's existing inference-error
  fail-closed path publishes a stop.
- Output remains `/control/command/control_cmd` with unchanged message type.

## Evaluation

1. Synthetic tests for stopping distance monotonicity, missing speed, side
   conflict, release, bilateral pinch and clear-scene identity.
2. Offline sequential replay of seed 2032 to verify that the crash precursor is
   classified before the historical 1.80 m frontal transition and that the
   selected side matches the published steering.
3. ROS package build and complete TinyLidarNet test suite.
4. Closed-loop unseen-seed run.  Only a strict successful outcome may enter the
   dataset pipeline; otherwise record the counterexample and redesign.

## Offline acceptance evidence

The initial hard-commit implementation was rejected before closed-loop use:
575/6,479 samples became side-conflict braking.  The bounded-hazard plus
two-sample switch design preserves the historical active count (1,557 samples)
and changes acceleration in only 13 samples relative to the historical replay.
It changes steering by at least 0.02 rad in 925 samples; p95 absolute change is
0.09772 rad.

At the seed 2032 failure precursor it acquires the left candidate at 9.78 m,
publishes left steering at 9.55 m and starts bilateral-pinch braking at 9.34 m.
The historical teacher first changed toward that side at 1.80 m.  This proves
earlier causal intervention in replay, not closed-loop success.

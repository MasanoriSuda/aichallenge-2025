# Design audit

## Observed causal chain

1. Sequence 27 is admitted as a current-world-certified ShiftOut artifact.
2. The artifact cursor advances continuously from 0.12 s.
3. Retained revalidation compares its continuously integrated progress state
   with `MpcProblem::progress_origin_m`.
4. `progress_origin_m` is populated from `BicycleModel::s`.
5. `BicycleModel::update_states()` assigns `s` to cumulative progress at the
   selected waypoint; it is therefore a staircase rather than the vehicle's
   continuous along-course position.
6. The comparison crosses the 1.5 m threshold between waypoint updates,
   temporarily accepts when the selected waypoint advances, then rejects
   again while the artifact cursor keeps advancing.
7. Normal authority disappears and the external Stop authority brakes.

## Hypotheses

### H1: discrete/continuous progress semantic mismatch

Support:

- `BicycleModel::t2s()` hard-codes the along-frame component `t` to zero.
- `update_states()` sets `s = get_s_at_waypoint(wp_id)`.
- trajectory points are approximately 1 m apart.
- the frozen artifact alternates rejection and acceptance while keeping the
  same immutable sequence and published-plan clock.

Refutation:

- Log physical progress at the control origin and the artifact's physical
  progress (`theta + lag`). If those differ by more than the tolerance during
  every rejection, the rejection is real rather than a coordinate mismatch.

Confidence: high.

### H2: artifact clock is advanced from the wrong origin

Support:

- first publication begins at artifact cursor 0.12 s.

Refutation:

- the clock fields remain monotonic and match
  `first_cursor + current_control_origin - first_publish` in the frozen log.

Confidence: low; the frozen timestamps are internally coherent.

### H3: the planned velocity is physically infeasible

Support:

- at decision 1880 the old expected velocity is 4.82 m/s while the fresh
  control-origin velocity is 1.43 m/s.

Refutation:

- retained revalidation intentionally re-anchors velocity to the fresh
  control-origin state and rebuilds an exact continuation. This discrepancy
  is a consequence of Stop ownership after authority loss, not the first
  rejection.

Confidence: medium as a downstream amplifier, low as the first cause.

## Intended correction boundary

Progress continuity must compare physical along-course position at the same
control origin:

```text
current physical progress  = current course-frame progress + current lag
artifact physical progress = artifact theta + artifact lag
```

The circular lift still selects the lap. The exact continuation then rebuilds
the current Frenet state and retains wall/dynamic proof. This does not relax a
threshold; it removes a coordinate-semantic mismatch before applying it.

The correction should be isolated behind an explicit helper/contract and must
not change the legacy three-state MPC coordinate.

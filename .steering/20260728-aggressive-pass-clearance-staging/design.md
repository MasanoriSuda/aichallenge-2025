# Design

## Lateral zones

Use the existing parameters as three explicit lateral zones:

| Locked-target relative lateral separation | Policy |
|---|---|
| `>= vehicle_radius + prediction_margin` (`1.50 m`) | Full Pass speed may be released |
| `>= vehicle_radius` and `< 1.50 m` (`1.45-1.50 m`) | Keep committed Pass, reapply Pass closing-speed cap |
| `< vehicle_radius` (`< 1.45 m`) | Target remains a front-overlap hazard; existing braking applies |

The body-clearance exception applies only when all of these are true:

- phase is `Pass`;
- target is the locked execution target;
- full-clearance exclusion has already latched;
- current course-relative lateral separation is at least the combined body width.

The normal `1.50 m` clearance remains responsible for acquiring the latch and releasing speed.
This prevents a `ShiftOut` or an unlatched maneuver from using the smaller body boundary.

## Wall clearance

Change `v2x_overtake_line_min_wall_clearance` from `0.20 m` to `0.10 m`.
With the current `1.45 m` kart width this makes the execution requirement approximately:

`1.50 + 0.725 + 0.10 = 2.325 m`

which is aligned with the `2.32 m` candidate corridor within rounding.

## Observability

Add `body_clear` to the periodic V2X debug line and the locked-target clearance values to V2X
state-transition logs. These are existing low-rate/state-transition logs, so no new high-rate
logging path is introduced.

# Design

## Problem

The current controller replans Mission metadata, but its actual lateral path is
still essentially:

1. smooth ShiftOut to one `pass_lateral`,
2. hold that offset,
3. smooth Return.

Consequently, changing curvature and narrowing wall bounds are handled mostly
through Mission replacement and Recovery.  This is discrete tactical replanning,
not continuous trajectory re-optimization.

## First-stage receding-horizon planner

For each MPC waypoint, build a convex lateral interval from:

- the live wall corridor plus the existing planning clearance;
- the locked target's predicted longitudinal-overlap window;
- the selected pass-side separation constraint while overlap is possible; and
- a trust region around the already validated Mission horizon.

Within those intervals, minimize a quadratic objective over all lateral
samples:

- deviation from the admitted Mission path;
- deviation from the prior cycle's warm start;
- first-difference (lateral slope) cost;
- second-difference (lateral curvature) cost; and
- a bounded preference for the outside of upcoming curves.

The optimizer uses projected coordinate iterations, so every intermediate and
final sample remains inside its hard interval.  The result is then checked by
the existing static-map footprint and lateral-reachability logic.  Any failure
returns the unmodified Mission horizon.

## Tactical boundary

Left/right homotopy selection remains discrete: it cannot be made continuous
without passing through the target's occupied interval.  This change makes the
trajectory *within the selected homotopy* continuously optimized. Existing
early/opponent side replan remains responsible for switching homotopy before
the no-return point.

## Runtime flow

```text
Mission candidate / pass side
        |
        v
legacy validated horizon  ---- fallback ----------------------+
        |                                                    |
        v                                                    |
live wall + predicted target side bounds                     |
        |                                                    |
        v                                                    |
projected receding-horizon lateral optimization              |
        |                                                    |
        v                                                    |
static footprint + reachability validation -- failure -------+
        |
        v
existing 40 Hz MPC target_ey / target_epsi
```

## Initial tuning

- Enabled for ShiftOut and Pass only.
- 12 projected iterations per control cycle.
- Maximum deviation from the admitted path: 0.35 m.
- Curve-outside preference: 0.20 m on significant curvature.
- Reference / warm-start / slope / curvature weights are exposed in YAML.

These are deliberately bounded: the purpose is to remove the single-offset
path limitation without bypassing the proven Mission and safety layers.

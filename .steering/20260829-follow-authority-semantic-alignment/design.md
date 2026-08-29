# Design: Follow authority semantic alignment

## Cause-to-change mapping

`Behavior` is a tactical observation.  `Action` and canonical intent describe
the controller which actually owns the command.  The current resolver merges
those concepts:

```text
Behavior Follow OR Follow cap
  -> Action Follow
  -> canonical Follow QP
```

This selects side-specific Follow escape geometry even when the command is
otherwise pure racing-line Cruise.  The resulting hard Follow problem can
starve canonical publication despite a distant opponent.

The corrected ownership is:

```text
Follow cap owns longitudinal control
  -> Action Follow -> canonical Follow QP

Behavior says Follow but no Follow cap owns control
  -> Action Cruise -> canonical Cruise QP
  -> current target tube remains in the same MPCC
```

This is not a fallback from a failed Follow solve.  The immutable authority
request selects exactly one formulation before solving.

## Preserved safety path

`resolve_dynamic_obstacle_contract()` already accepts Cruise plus a complete
current target tube.  The normal snapshot therefore keeps the opponent rows,
exact timed-obstacle proof and terminal successor proof.  Only the false
Follow semantic owner is removed.

## Excluded scope

- Follow candidate geometry and solver behavior when a real Follow cap is
  active.
- Overtake Mission admission or side selection.
- Parameter tuning and Recovery behavior.

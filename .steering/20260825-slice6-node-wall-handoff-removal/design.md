# Design

## Before

```text
canonical MPCC command + certificate
  -> node-level solver/overtake/dynamic wall gate
  -> hold/decelerate/replan/replace axes
  -> final publisher
```

This creates a second normal authority after the canonical decision. Two of
the three gates are now unreachable; the solver-recovery gate is reachable but
duplicates the canonical current-world proof.

## After

```text
canonical MPCC command + certificate
  -> final publisher

canonical failure
  -> typed Emergency or bounded solver-failure supervisor

physical collision/stuck
  -> Stuck/gear/reverse Recovery
```

`executed_solution_wall_hold_active` remains inside the canonical controller
boundary. The node publisher only arbitrates canonical normal versus explicit
supervisor classes; it does not classify another normal path.

## Deletion accounting

- Normal authority added: 0.
- Safety supervisor deleted: 0.
- Configuration added or tuned: 0.
- Duplicate/dead normal owners removed: legacy wall authority resolver,
  solver handoff gate, active-overtake gate, DynamicEscape wall/exit gates and
  their publisher source classes.

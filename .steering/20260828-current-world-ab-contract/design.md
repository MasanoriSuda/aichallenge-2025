# Design: Current-world A/B contract

## Root cause

The first native comparison stopped before SQP for two architectural reasons:

1. the nominally stateless arm required `dynamic_obstacle_stages`, which are
   optional output of the persistent Mission pipeline;
2. successor viability required a 2.4-second receding horizon to contain the
   complete Return or a terminal state whose velocity interval already
   included zero.

Both conditions make the comparison measure Mission materialization rather
than the persistent-versus-stateless candidate.

## Current-world target horizon

The comparison reconstructs the selected target from immutable `ReplayWorld`:

```text
observation + control-origin age + stage times
  -> constant-velocity world positions
  -> projection on the recorded unwrapped course-frame window
  -> local progress/lateral stage tube
```

The QP disjunction uses physical center separation derived from the recorded
ego footprint, its margin and the recorded peer radius.  Exact swept-body
proof remains the final authority and uses the same world observation.

No Mission path, phase transition geometry, lease or runtime clock is read.

## Receding successor

- `Return`: the target encounter ends in-horizon and the terminal wall interval
  contains the racing line.
- `Replan`: the encounter continues, but current-world reconstruction remains
  valid and physical braking authority exists for the next bundle.
- rejection: neither continuation can be established.

`Replan` carries a contingency Stop intent.  It does not claim the current
horizon reaches zero speed; any future Stop trajectory still requires the
same solver and physical certificates before authority.

## Comparison boundary

Persistent A retains the captured semantic QP.  Stateless B replaces only the
candidate lateral/heading reference and target-stage tube.  Both use fresh
solver contexts and the common exact wall/dynamic proof.  No output can reach
the live controller.


# Design

## Causal chain under audit

```text
ShiftOut primary trajectory solves and is physically clear
  -> terminal Stop is required as recursive successor
  -> Stop candidate immediately applies maximum braking and one fixed
     lateral path-feedback target
  -> every fixed-target member crosses an infeasible wall/lateral region
  -> swept footprint contacts wall
  -> all otherwise-feasible arms lose normal authority
  -> external Stop executes the same wall-seeking zero-offset policy
  -> actual footprint wall violation and Recovery
```

The visible Stop/Recovery is downstream. The first shared rejection is the
terminal successor candidate family, not the normal ShiftOut trajectory.

## Observation-only comparison

Extend the pure Stop path-tracking request with an explicit lateral reference.
Its feedback error becomes:

```text
lateral_error = current_lateral - target_lateral
target_curvature = course_curvature
                 - lateral_gain * lateral_error
                 - heading_gain * heading_error
```

All existing production callers pass `target_lateral = 0`, preserving current
commands. A new architecture-comparison-only arm passes the immutable
`ContingencyStopIntent::hold_lateral_m` produced by the same candidate.

The declared target and 128-target physical scan refute the initial hypothesis.
A second audit arm therefore solves a Stop with the same seven-state SQP.  It
starts only after replaying the already-selected command to the exact next
publisher boundary, then requires terminal velocity zero.  Its exact nonlinear
trajectory is checked against the unchanged wall and dynamic worlds.  No audit
arm has a Store, mailbox, command or publisher API.

## Promotion boundary

The frozen comparison rejects all fixed-offset suffixes and accepts the causal
seven-state Stop.  A separate Slice may therefore replace the one canonical
Stop candidate generator with a seven-state Stop artifact.  It may not add a
second Stop authority or retain the old fixed-policy suffix as a fallback.
This audit does not perform that promotion.

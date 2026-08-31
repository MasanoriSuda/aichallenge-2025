# Design: Stop stationary trajectory contract

## Root cause

`ExactPhysicalExecutionTrajectory` is a temporal artifact, but its validator
unconditionally treats `path_distance_m` as a strictly increasing spatial
axis.  During maximum braking the serialized command can reach zero speed
before its publisher interval ends.  Time and actuator-response state still
advance, while path distance correctly remains constant.  The common strict
distance rule rejects that valid Stop suffix, removes the certified terminal
successor and propagates into global normal-authority loss.

## Considered repairs

1. Add epsilon to repeated distance. Rejected: this fabricates motion and can
   invalidate physical wall provenance.
2. Drop the stationary samples. Rejected: this no longer proves the complete
   serialized command interval or its steering-response state.
3. Relax every exact trajectory to nondecreasing distance. Rejected: this
   weakens normal MPCC evidence and can hide a stalled candidate.
4. Add an immutable Stop-only stationary-suffix contract. Selected.

## Selected contract

The exact temporal artifact carries an explicit
`stationary_path_suffix_allowed` property.  It is false by default and is set
only by the maximum-braking Stop builder.  Equal adjacent path distances are
accepted only when:

- the property is set;
- both adjacent velocity samples are inside the already sealed physical
  zero-velocity tolerance; and
- distance never regresses.

All normal execution artifacts preserve their former strict-distance rule.
The physical wall and dynamic-obstacle layers receive the actual repeated
pose/time samples, so the repair changes certificate semantics rather than
geometry or authority policy.

# Design

## Root cause

`evaluate_overtake_line_horizon()` first projects nominal lateral samples onto
the acceleration-reachable interval.  It records that history in
`lateral_accel_limited`.  The later coupled full-footprint search can accept a
profile whose measured maximum acceleration is below the configured limit and
stores that accepted maximum in `max_required_lateral_accel`, but it does not
erase the historical flag.

The Pass-entry gate interpreted the historical flag as current physical
infeasibility.  Consequently the log could simultaneously report an accepted
maximum of 4.73 m/s2 and claim that the 6.0 m/s2 limit was exceeded.  However,
the projection proves only a short executable prefix; it does not prove that
the persistent Pass/Return geometry remains viable.

## Correction

Add a pure `resolve_pass_entry_execution_profile()` contract resolver.  It
separates:

- physical execution infeasibility,
- accepted-profile lateral-acceleration infeasibility,
- wall adjustment,
- an executable profile that used acceleration projection, and
- an executable unmodified profile.

The controller uses this resolver only at the ShiftOut-to-Pass physical gate.
An unmodified exact profile may authorize Pass.  A projected profile is
classified as `ProjectionRequiresSuccessorReplan` and keeps the gate closed
until a current-world successor is proved.  Existing full-Mission admission
and runtime wall contracts keep their current strictness.

## Safety invariant

A projected prefix is never promoted directly to Pass.  Pass is available
only when the exact accepted successor is physically feasible, inside the
unchanged acceleration bound and requires no wall adjustment.

## Rejected experiment

The first implementation treated a projected 4.73/6.0 m/s2 prefix as Pass
authority.  Dynamic run `output/20260830-020321/d1/autoware.log` produced two
`ShiftOut -> Pass` transitions, but the retained Mission later rebuilt its
original geometry.  Episode 6 accumulated lateral-acceleration and static-wall
physical failures before `actual footprint wall margin violated`; episode 2
also entered Return and then `actual footprint intersects static wall`.

This refutes direct promotion.  The required future structural correction is
an immutable, current-world Pass/Return successor bundle derived from the
projected prefix, not a looser gate.

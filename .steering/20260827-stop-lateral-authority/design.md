# Design: moving Stop follows the base path

## Root cause

The Stop authority boundary correctly removed the legacy normal solve, but it
also replaced the whole lateral sequence with one constant value copied from
the previous command. At racing speed, maximum braking still requires several
metres. A steering command that was valid at the instant SafetyBrake fired is
not a valid invariant over that distance, especially across changing course
curvature.

Stop shadow computation does not repair this. It intentionally stays
shadow-only, and the final Stop command ignores its steering. Promoting that
shadow would also be incorrect because its longitudinal trajectory differs
from the emergency command.

## Repair

Keep Stop as an external supervisor and replace its lateral zero-order hold
with a deterministic emergency lateral policy:

1. If the vehicle is moving and the current reference-path steering target is
   available, rate-limit the previous command toward that target.
2. If the path target is unavailable, rate-limit toward neutral.
3. Once speed is zero, hold the current bounded command to avoid stationary
   steering chatter.

The path target reuses the existing spatial path-feedback law and lateral-
acceleration envelope. The transition reuses the existing physical
steering-rate limiter. Therefore the change introduces no new gains or second
normal solver.

## Authority boundary

The resulting command is still typed `Stop`, uses unresolved normal
formulation, requests zero target speed, and receives maximum braking in final
publication. No normal canonical command or execution artifact is published
or marked executed. The normal shadow remains warm solely for the later
atomic Stop-to-normal handoff.

## Alternatives rejected

- Continue holding steering until standstill: dynamically falsified by the
  wall-contact sequence.
- Publish the latent ShiftOut shadow steering: its speed/progress proof does
  not describe the overridden Stop longitudinal command.
- Add a nominal Stop MPCC immediately: this is a larger architecture change
  and creates a new production intent before a measured need beyond emergency
  path tracking.
- Neutralize unconditionally: erases required curvature on a bend and repeats
  an older solver-fallback failure mode.

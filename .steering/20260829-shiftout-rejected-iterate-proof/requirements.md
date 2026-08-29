# Requirements: ShiftOut rejected-iterate proof

## Objective

Classify frozen ShiftOut fingerprint `9845010060330222052` without changing
production authority or numerical settings.  Determine whether OSQP's
maximum-iteration result contains a physically certifiable primal which is
currently discarded before the architecture comparison can inspect it.

## Repaired evidence invariant

A replay-ready solver failure must preserve its finite rejected primal as
diagnostic evidence.  It must never expose that primal as a `SolveResult`,
certified artifact, Store entry or command.

## Constraints

- No solver iteration, tolerance, scaling, weight or clearance change.
- No timeout, lease, grace, retry or fallback.
- No production authority or publisher change.
- The ordinary solved-result contract remains unchanged.
- Exact trajectory, wall, dynamic-obstacle and terminal proofs remain the
  classification authority.

## Definition of done

1. A maximum-iteration finite primal is available only through a diagnostic
   field.
2. The architecture CLI can replay and proof-check that diagnostic iterate.
3. Tests prove it cannot be mistaken for a solved result.
4. The frozen fingerprint is classified or remains explicitly `Unknown`.

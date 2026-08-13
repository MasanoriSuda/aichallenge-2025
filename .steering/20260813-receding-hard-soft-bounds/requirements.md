# Requirements

## Purpose

Eliminate avoidable `optimized horizon escaped hard bounds` Recovery transitions by distinguishing execution-critical constraints from trajectory-continuity preferences.

## Scope

- Keep wall bounds and active opponent-separation bounds hard.
- Treat the per-cycle Mission trust region as soft during post-validation repair.
- Reuse the existing predicted-overlap confirmation before revoking body-clear Pass separation release.
- Report whether a repaired trajectory crossed only the soft trust region or a true hard bound.

## Constraints

- Actual wall infeasibility, actual body overlap, emergency front risk, target jumps, and blocked execution corridors remain hard failures.
- No reverse/stuck-recovery changes.
- No ROS 2 interface or result-schema changes.


# Requirements

Retire the runtime wall-escape prefix's authority to destroy a canonical
Overtake Mission.

## Evidence

In `output/20260824-132703`, generation-1 ShiftOut had certified canonical
retained authority. A runtime wall preview then entered its action band. The
legacy centerward-prefix preflight rejected the reference for lateral
acceleration and immediately executed:

```text
runtime wall escape prefix unavailable
-> Mission generation invalidated
-> ShiftOut -> FollowPrepare / DynamicMissionWait
```

The decision trace reported `hard_fault=0`, and alternate-side evaluation was
still in flight. Canonical current-world proof subsequently detected the real
wall contact and selected explicit Emergency. The prefix heuristic therefore
acted as a second Mission-viability owner before the canonical safety boundary.

## In scope

- Remove `RuntimeWallPreplanAction::ExitCurrentMission`.
- Preserve phase, target, side and generation when a legacy centerward prefix
  is unavailable.
- Keep runtime wall warning telemetry, same-side/cross-side candidate
  generation and accepted prefix replacement.
- Keep actual wall/contact/map hard faults under the existing external
  supervisor and canonical current-world certificate.
- Add deletion gates and deterministic resolver tests.

## Out of scope

- Wall, lateral-acceleration, TTC, timing or clearance tuning.
- A grace period, timeout, retry, lease, cooldown or fallback command.
- Changing canonical physical proof or Emergency behavior.
- Rejoin, Stuck/Reverse Recovery or parameter tuning.

## Definition of done

- Prefix unavailability cannot invalidate or transition a canonical
  ShiftOut/Pass/Return Mission.
- Accepted prefix and speed-preserving Return paths remain available.
- Hard wall faults remain excluded from runtime preplan and fail closed through
  the independent supervisor.
- Static/package tests and a bounded dynamic run pass.

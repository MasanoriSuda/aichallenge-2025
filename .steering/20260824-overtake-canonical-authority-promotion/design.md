# Design

## Single authority resolver

Add one controller boundary for canonical Overtake intents:

1. Consume the latest async plan only after current-world retained proof and
   canonical selector completion.
2. Publish through `canonical_normal_control` only when the selected command,
   problem, solution, immutable plan, cursor and prediction are complete.
3. Otherwise publish through `canonical_normal_emergency_stop`.

The control callback never solves this formulation. The first dynamic Gate
proved that duplicating the async solve in the callback starved the worker,
created maximum-iteration failures and violated the 40 Hz runtime contract.
No retry, grace, lease or second normal formulation replaces missing async
evidence.

## Physical certificate ownership

Fresh canonical selection proves the exact five-state Frenet pose sequence,
including lag and heading, against the production static-wall grid with the
required lateral wall clearance. Retained selection reconstructs and proves
the remaining exact pose sequence against the current pose, course frame,
target observation and corridor before every adoption.

The older active-overtake wall admission receives only world x/y samples. It
reconstructs yaw from adjacent points and applies a generic all-direction wall
proximity metric. It therefore cannot validate or reject the canonical
certificate: it is a different trajectory and clearance contract. Canonical
normal or canonical Emergency output bypasses that legacy normal-authority
gate. Emergency, current-pose hard contact, Recovery and command fail-safe
remain independent overrides.

## Deletion boundary

`get_control()` returns from this resolver for ShiftOut, Pass and Return before
declaring or entering the legacy-shaped `dec` path. The old conversion,
circuit/reentry gate and three-state solve remain temporarily for intents not
yet covered by canonical Overtake, but are unreachable from this scope.

This Slice deliberately does not remove tactical Mission/DP inputs. They remain
problem inputs; they no longer own the published Overtake command.

## Invalid DynamicWait boundary

DynamicWait is a continuation of a committed ShiftOut or Pass only while it
owns an executable lateral prefix. If that prefix disappears, the intent
resolver deliberately returns `unknown`; this is an invalid handoff, not
permission to resume racing-line or legacy MPC authority. Such a state returns
the existing canonical Emergency before the old normal formulation block.

This does not invent a replacement path, timeout or retry. It makes the
already-invalid authority state fail closed and preserves the evidence needed
for the tactical layer to rebuild or leave the Mission.

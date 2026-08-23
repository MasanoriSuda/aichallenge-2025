# Requirements

## Objective

Produce exact Overtake canonical plans outside the 40 Hz control callback and
admit them in shadow only after exact identity and current-world revalidation.

## Root cause addressed

The live Overtake path currently solves five-state MPCC synchronously, builds a
canonical artifact only for telemetry, then converts the same result to the
legacy layout. Fresh canonical results have no independent producer lifetime,
and retained plans cannot cover the circuit/reentry/three-state gaps observed
in `output/20260824-031752`.

## Scope

- Add a reusable exact-intent async context lifecycle.
- Give Overtake its own temporary shadow solver/plan lifecycle.
- Snapshot the already constructed Overtake problem and solve it in one
  latest-only worker.
- Validate worker identity against current intent/target before use.
- Require current-world proof before storing or selecting any incoming plan.
- Preserve all current production command behavior for the dynamic gate.

## Non-scope

- No publisher connection or authority promotion.
- No deletion of production conversion/circuit/reentry/three-state paths yet.
- No solver, wall, vehicle, timing or behavior parameter changes.
- No timeout, lease, retry, fallback or feature flag.

## Acceptance

- Failure-first tests prove phase changes reset async context even when target
  and intent generation are unchanged.
- Unsupported and incomplete contexts fail closed.
- Follow continues to use the same generic lifecycle semantics.
- Overtake worker result cannot cross intent, target or context epochs.
- Focused tests, package tests and build pass.
- A dynamic shadow gate measures submitted/completed/current-world-ready and
  retained coverage before any production promotion.

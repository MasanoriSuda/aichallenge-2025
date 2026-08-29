# Requirements

## Objective

Repair the producer/admission disconnect which prevents a certified
current-world opposite-side trajectory from reaching the active Overtake
Mission replacement boundary.

## Frozen evidence

- Baseline: `b2d646a9`
- Dynamic run: `output/20260830-031429/d1/autoware.log`
- Failure episode: Mission generation 1, ShiftOut side `-1`, waypoints 58--60.
- The live tactical layer repeatedly requested side `+1`, but runtime
  replacement reported `proposal=0` and retained side `-1` until the exact
  wall monitor rejected it.
- Frozen architecture snapshot:
  `aichallenge/mpcc_architecture_snapshots/000000001799-77540dfa305ff570-`
  `shiftout-side-negative-physical-proof-physical-proof-rejected/snapshot.yaml`
- On that immutable world, the current side failed while the bounded
  production candidate population produced an accepted, exactly wall- and
  opponent-certified side `+1` artifact.

This refutes physical infeasibility and a general single-SQP limitation. The
first invalid edge is the lifecycle/admission bridge: active replacement asks
for the MPCC-lite runtime candidate, while causal Gate A is still built only
from the new-entry branch selection.

## Constraints

- Do not change solver settings, wall/vehicle clearance, speed policy, leases,
  timeouts, retries, fallback, or Recovery policy.
- Do not bypass Gate A or weaken exact current-world proof.
- Idle/new-entry admission continues to use pre-entry branch selection.
- Active ShiftOut/Pass replacement must use the actual canonical runtime
  replacement request; it must not fall back to the new-entry producer.
- A current-world certified trajectory and its target, side, generation, and
  tactical source identity must still travel atomically.
- Production authority remains unchanged until dynamic verification.

## Definition of Done

- A pure, tested resolver assigns exactly one Gate A tactical input owner.
- Idle resolves only from pre-entry selection.
- Active execution resolves from the runtime same/cross-side MPCC candidate
  using the same precedence as the execution layer.
- Active execution cannot silently reuse pre-entry geometry.
- The selected candidate is re-certified from the current serialized command
  before a proposal is emitted.
- Build, package tests, source-contract tests, and `make dev2` pass.
- Dynamic logs show either an accepted opposite-side Gate A replacement or a
  newly classified current-world proof/admission rejection instead of
  `proposal=0` while an executable runtime candidate exists.

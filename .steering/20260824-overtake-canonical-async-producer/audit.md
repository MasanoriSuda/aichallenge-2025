# Overtake canonical async producer audit

## Evidence entering the Slice

`output/20260824-031752` proved that numerical row admission is no longer the
first blocker. The exact fresh chain completed on 198 cycles, but retained
coverage was 0/58 attempts and production used 102 circuit/reentry/three-state
fallback cycles. Final Overtake decisions remained legacy or wall-hold.

## Root cause

The canonical artifact is derived from a synchronous production solve and then
discarded as telemetry. The live execution lifetime remains owned by the
converted legacy vector and its formulation arbitration. Consequently there is
no independently advancing canonical producer/store that can replace an
unavailable cycle with a current-world-certified plan of the same formulation.

## Failure-first dynamic evidence

The first `make dev2` gate (`output/20260824-040144`) reached Overtake episode
1 and submitted 82 jobs. The worker started/completed 81 jobs without an
exception, but published zero plans and every observed result ended as:

```text
eligibility-reject/intent-not-overtake-execution
```

The live authority was `ShiftOut`; the immutable job identity and sealed
problem context also carried `ShiftOut`. The rejection was therefore not a
tactical, numerical, wall, or timing failure.

## Root-cause refinement

The worker fresh-chain rebuilt its result and `MpccProblemContext` through
`current_control_intent()` on a tactical `MPC` clone. That clone intentionally
does not own the live authority trace, so it fell back to `Cruise`. The worker
therefore discarded a valid sealed ShiftOut job using incomplete clone state.

Copying another mutable authority field into the clone would preserve two
sources of truth and make later omissions possible. The structural repair is
to make the complete sealed context a required argument of the whole worker
fresh-chain and forbid intent/context re-derivation there.

The second gate confirmed the first half of that repair: the intent rejection
disappeared. It then exposed the same defect in the worker entrance check. The
check rebuilt the whole fingerprint with `make_problem_context()` on the clone,
so all 122 completed jobs ended as `context-reject` even though the immutable
problem and sealed context came from the same live decision. The required
repair is one shared context sealer that accepts authoritative semantic
provenance and recomputes only problem-owned structure (geometry, horizon,
formulation and schemas).

The third gate passed both sealed-context checks and exposed the next legacy
boundary. `ShadowWarmStartIdentity` completeness still hard-coded only
Track/Cruise/Follow, so every ShiftOut job was rejected as
`warm-context-reject/invalid-current-context` before solve. Canonical normal
intent support already has one contract function; warm-start completeness must
use that same contract rather than retain a second, older intent list.

## Required invariant

An Overtake plan may enter the shadow selector only if one immutable job names
its exact intent, intent generation, target observation, problem fingerprint
and context epoch, and the live controller subsequently proves that plan
against the current world. Worker age alone is never acceptance evidence.

## Conclusion

The fourth gate (`output/20260824-043223`) satisfied the producer invariant
through both ShiftOut and Pass. The worker completed 157 jobs, built 155 exact
physical canonical chains and live revalidation accepted/stored 123 incoming
plans. No worker exception, identity rejection, submission rejection, snapshot
failure, callback overrun, or any of the three root-cause rejection signatures
remained.

This accepts the async producer Slice but does not authorize production
promotion. Across 158 eligible shadow cycles, 34 did not obtain current-world
selection: 23 lacked the required current course-frame window, 9 violated the
new current stage corridor, 1 lacked corridor horizon and the initial cycle had
no completed result. These are fail-closed proof gaps, not reasons to reconnect
legacy authority implicitly. The next Slice must audit current-world coverage
from its earliest missing provenance before publisher connection and atomic
legacy deletion.

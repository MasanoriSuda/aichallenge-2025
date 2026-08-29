# Requirements: DynamicWait canonical-intent handoff

## Frozen baseline

- Baseline commit: `bfaf7333`
- Dynamic run: `output/20260830-004030`, Domain 1
- Representative decision: 1473 near waypoint 67

## Observed causal sequence

1. An admitted ShiftOut Mission enters tactical FollowPrepare/DynamicWait after
   the current-side prefix becomes temporarily unavailable.
2. `resolve_canonical_execution_identity()` correctly preserves target `d2`,
   generation 1, side 1 and the interrupted ShiftOut phase.
3. The optional legacy forward prefix is not active in the entry decision.
4. `resolve_canonical_control_intent()` ignores the preserved canonical
   execution identity and requires `DynamicWaitPrefix` to be both lateral owner
   and path source.
5. The intent becomes Unknown, so the seven-state current-world producer is
   not invoked and Emergency Stop is published at 4.72 m/s.
6. Eighteen decisions later the optional prefix appears and the same Mission is
   accepted as ShiftOut, proving that the first stop was an ownership/update
   ordering defect rather than a physical impossibility.

## Root cause

The 2026-08-23 DynamicWait intent-provenance implementation made a legacy
forward/hold prefix the prerequisite for canonical intent. The later
current-world canonical publisher demoted that prefix to an optional reference,
but the old prerequisite and telemetry owner survived. DynamicWait therefore
has two incompatible definitions of lateral authority.

## Constraints

- Do not restore the optional prefix as command authority.
- Do not add a resume rule, lease, grace, timeout, retry or fallback.
- Do not change solver settings, weights, clearance or proof tolerance.
- A valid canonical execution identity selects only the semantic intent; it
  never bypasses current-world wall, opponent or terminal proof.
- Malformed or absent execution identity must remain fail-closed.
- Delete the obsolete prefix-owner contract in the same Slice.

## Definition of done

- DynamicWait preserves ShiftOut/Pass intent from the canonical execution
  identity, independent of optional prefix availability.
- DynamicWaitPrefix is no longer represented as a lateral command owner.
- Mission identity, origin phase and side must be coherent before intent is
  accepted.
- The current-world seven-state producer and all exact proofs remain the only
  route to normal publication.
- Focused tests, source contracts, package tests and build pass.
- Dynamic acceptance observes no `dynamic-wait-without-lateral-authority`
  Emergency at DynamicWait entry, while malformed identity still stops.

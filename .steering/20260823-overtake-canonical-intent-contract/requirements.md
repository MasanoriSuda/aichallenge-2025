# Overtake canonical intent contract

## Purpose

Remove the canonical-contract defect that rejects every physically certified `ShiftOut`, `Pass`
and `Return` five-state artifact as an unsupported normal intent.

## Root cause

The fresh-chain shadow proved that 352 exact Overtake artifacts passed primal normalization,
actuation/trajectory extraction and swept physical wall certification. The shared canonical plan
contract then rejected every artifact because `canonical_normal_intent_supported()` contains only
`Track`, `Cruise` and `Follow`.

Adding the three Overtake intents alone would expose a second contract hole: the current problem
completeness rule requires a target only for `Follow`. A canonical Overtake plan must never be valid
without target ID and target observation generation.

## Scope

- Define the exact set of supported canonical normal intents.
- Define which supported intents require target provenance.
- Admit `ShiftOut`, `Pass` and `Return` only with complete target provenance.
- Update canonical candidate, plan and retained-provenance tests.
- Rerun the existing fresh-chain shadow without changing production authority.

## Non-scope

- No Overtake production-authority promotion.
- No retained Overtake plan or current-world revalidation implementation.
- No removal of three-state fallback or legacy conversion yet.
- No solver, clearance, cost, horizon, timeout, lease or cooldown tuning.
- No Mission, target-selection or path-geometry change.

## Acceptance

- `Track`, `Cruise`, `Follow`, `ShiftOut`, `Pass` and `Return` are the only supported normal intents.
- `Follow`, `ShiftOut`, `Pass` and `Return` require target ID plus target observation provenance.
- Exact-intent canonical candidates for all three Overtake phases are accepted.
- Cross-intent candidates fail as `IntentMismatch`, never by borrowing another phase's plan.
- Replay advances physically certified Overtake artifacts beyond `UnsupportedIntent`.
- Production output remains shadow-only for Overtake.

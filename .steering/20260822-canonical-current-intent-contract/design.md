# Design

## Observed contract gap

`CanonicalNormalAuthorityRequest` currently carries only current decision/time and
fresh/retained candidates.  Candidate qualification checks that the candidate's
own sealed problem is Track or Cruise, but it cannot compare that intent with the
current supervisor intent.

This allows the following source-level counterexample:

```text
old certified problem intent = Track
current supervisor intent    = Cruise
current execution proof      = certified for current decision
fresh candidate              = unavailable
retained candidate           = accepted
```

The visible command problem would appear at future authority promotion, but the
producer is the incomplete authority request contract.  Revalidation is not a
substitute for current intent provenance.

## Causal classification

- Root cause: current supervisor intent is absent from the authority request.
- Detection gap: candidate qualification validates only the candidate's historical
  problem intent.
- Contributor: retained candidates intentionally preserve their original solver
  decision identity, so a current physical proof alone is insufficient.
- Mask: none in the pure selector; current runtime shadow status prevents execution.
- Recovery: Emergency Stop is already the fail-closed destination and is unchanged.

## Hypotheses and falsifiers

| Hypothesis | Evidence | Falsifier | Confidence |
|---|---|---|---|
| A retained plan can cross an intent transition | Request has no current intent; qualifier only checks candidate intent | Existing selector test rejects Track candidate under Cruise request | High |
| Current decision proof implicitly proves intent | Proof contains decision ID and physical flags only | Proof or fingerprint binds current supervisor intent | High that this is false |
| Caller-only guarding is sufficient | Current shadow caller invokes only a fresh candidate | Every future/final caller is structurally forced through an identical guard | Low; not encoded in API |

## Options

### A. Guard only at the controller caller

Small diff, but preserves an invalid reusable API and permits the final publisher or
future retained path to omit the guard. Rejected.

### B. Add current intent to the selector request and require exact intent equality

Repairs the producer, preserves one fail-closed destination and adds no normal
authority. Selected.

### C. Permit Track/Cruise interchange

Could retain more coverage, but Track and Cruise can have different target/cost
semantics. Re-certifying only geometry cannot prove objective compatibility.
Rejected until a separate equivalence proof exists.

## Implementation

1. Add `current_intent` to `CanonicalNormalAuthorityRequest`.
2. Reject a request whose current intent is not Track or Cruise.
3. Add `IntentMismatch` candidate rejection and require candidate problem intent to
   equal the current intent for fresh and retained candidates.
4. Pass the already-computed shadow `result.intent` at the sole runtime caller.
5. Update aggregate initializers in tests explicitly.

## Branch/configuration delta

- New normal authority branches: zero.
- New configuration/feature flags: zero.
- Deleted masks/bypasses: none; the missing invariant is added at its producer.
- Remaining legacy authority: unchanged because this slice is shadow-only.

## Rollback

Rollback target: `f9272d0e0e92582b69f5f4016ce38f60c5d6cdbf`.

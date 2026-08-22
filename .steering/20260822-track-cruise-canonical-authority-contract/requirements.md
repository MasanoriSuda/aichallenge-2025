# Track/Cruise canonical authority contract

## Baseline

- Branch: `develop_july`
- Baseline commit: `f85f671`
- Preserve the unrelated user change `aichallenge/result-summary.json`.

## Purpose

Prepare Slice 3 without promoting runtime authority. Encode the only permitted Track/Cruise normal
selection chain as a deterministic contract:

```text
fresh certified five-state control plan
-> bounded retained certified five-state control plan
-> explicit Emergency Stop
```

There is no three-state, legacy MPC, direct-control or deceleration-fallback candidate in this
contract.

## Confirmed gaps in the current implementation

1. Track/Cruise five-state output is stored only as a shadow `ActuationProposal`; it is not recorded
   as a complete `CertifiedMpccSolution` plus executable control plan.
2. The live solve orders legacy production before Track/Cruise shadow, so shadow cannot prevent a
   legacy command from consuming the last reachable wall-clear prefix.
3. Extended solver failure falls through to three-state/legacy solve and then to
   `safe_failure_control()`; this violates Slice 3's same-formulation fallback rule.
4. Existing retained dynamic-escape state is target/side-specific and cannot be reused as a generic
   Track/Cruise canonical store.
5. `CertifiedMpccSolution` is certificate metadata only. An authority selector must also require
   an explicitly available executable control plan.

## Required contract

- Accept only complete Track/Cruise five-state problem/solution identities.
- Fresh candidate must belong to the current decision.
- Retained candidate may keep its original decision identity, but must remain certified, unexpired
  and have executable control stages remaining.
- Reject a valid certificate if its executable plan is absent.
- Reject all non-five-state formulations.
- Resolve to Emergency Stop when neither canonical candidate is admissible.
- Provide a stable reason enum/string for decision tracing.

## Non-scope

- No runtime authority promotion.
- No command publication or config change.
- No legacy branch deletion yet.
- No parameter, wall-clearance or solver tuning.
- No reuse of a warm-start vector as an executable certified plan.

## Exit gate

- Failure-first tests prove that the current contract has no canonical selector.
- Deterministic tests cover fresh, retained, expired, noncanonical, mismatched and missing-execution
  cases.
- The selector exposes no legacy fallback input or output.
- Full build and package tests pass.
- Runtime remains `authority=shadow, selected=0`.

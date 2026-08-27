# Requirements: Stop-to-normal atomic authority handoff

## Objective

Remove the one-cycle authority gap observed when SafetyBrake releases to a
normal Follow intent by making the actually published Stop authority visible
to the canonical atomic-admission boundary.

## Failure-first evidence

- In `output/20260827-211306/d1/autoware.log`, decision 1088 published an
  explicit `Stop` supervisor command after `Follow -> SafetyBrake`.
- The Stop shadow Follow producer correctly rejected that observation with
  `initial-hard-gap-violation`; producing a normal Follow command while the
  hard gap was violated would have been invalid.
- At decision 1093, SafetyBrake released to Follow before the asynchronous
  Follow artifact was available. Atomic admission compared Follow only with
  the last *normal* published intent, Cruise, and reported
  `no-current-world-authority`.
- Decision 1093 consequently emitted a generic Follow Emergency. Decision
  1094 accepted the freshly certified Follow artifact and resumed normally.
- The physical command never lacked a safe stop, but the authority ledger did:
  the actually published Stop was absent from transition admission.

## Invariant

A serialized Stop command remains the effective authority until a certified,
current-world normal successor is atomically admitted. Stop retention must not
depend on elapsed time, artifact age, or reuse of an obsolete normal plan.

## Constraints

- Do not relax SafetyBrake, hard-gap, wall, solver, or timing parameters.
- Do not add a timeout, grace period, lease, retry, or additional fallback.
- Preserve the last published normal intent independently because Stop shadow
  successor selection needs the interrupted normal semantic.
- Only an exactly serialized Stop or canonical normal publication may update
  the published-authority ledger.
- Keep generated result files uncommitted.

## Definition of Done

- A deterministic contract test proves that Stop can be retained when a
  proposed normal intent has no current-world authority.
- The runtime distinguishes the last published wire authority from the last
  published normal intent.
- Stop-to-Follow publishes Stop until Follow joins, then switches atomically.
- Focused tests, all package tests, and `make autoware-build` pass.
- A bounded `make dev2` shows no generic normal-authority Emergency between
  SafetyBrake release and the first admitted normal successor.

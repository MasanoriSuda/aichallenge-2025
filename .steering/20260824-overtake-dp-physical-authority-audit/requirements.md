# Requirements

## Objective

Identify the first violated invariant behind the first post-entry Overtake
failure in `output/20260824-123452/d1/autoware.log` without changing racing
behaviour or tuning.

The accepted `ShiftOut` command is a certified five-state canonical command,
but the legacy Frenet-DP execution source expires after about 0.53 s and a
separate receding-horizon physical revalidation later invalidates the Mission.
The current log reports only the aggregate reason, so it cannot distinguish a
real future wall/kinematic infeasibility from a duplicate-authority/provenance
failure.

## Evidence boundary

- Branch: `develop_july`
- Baseline: `7da5205`
- Dynamic run: `output/20260824-123452`, Domain 1
- Last known normal event: decision 2384, generation 1, `ShiftOut`, canonical
  retained five-state command published at timestamp 1787542555.521.
- First authority degradation: DP source released as `source-stale` at
  1787542555.989.
- Visible Mission failure: `optimized horizon failed physical revalidation`
  at 1787542557.424.

## Invariants under audit

1. A physical rejection identifies the exact stage, path distance, profile
   lateral value, wall bounds, heading, speed and clearance contract that
   failed.
2. The selected canonical solution, its current-world proof, the lateral
   execution source and the Mission viability decision have traceable
   provenance.
3. A stale tactical/DP source is not confused with a physical wall failure.
4. Observation work must not add a normal authority, lease, timeout, retry,
   fallback, threshold or parameter change.

## Scope

- Receding-horizon physical-revalidation diagnostics.
- DP source and Mission generation provenance at the failure boundary.
- Canonical stored-plan identity at the same failure boundary.
- Deterministic formatting/source-contract tests.

## Non-scope

- Changing wall margins, lateral acceleration, horizon timing or solver
  settings.
- Keeping a stale DP source alive.
- Suppressing physical rejection or Recovery.
- Promoting a canonical plan to another execution owner before the cause is
  proven.
- Rejoin/Recovery migration.

## Acceptance

- The next reproduced failure names a physical failure cause and stage, not
  only `optimized horizon failed physical revalidation`.
- The same record contains Mission/DP/canonical-plan provenance.
- Existing tests and package build pass.
- Production commands and state-transition decisions remain unchanged.

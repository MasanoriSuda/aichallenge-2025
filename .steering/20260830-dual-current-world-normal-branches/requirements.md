# Requirements

## Baseline and evidence

- Baseline: `40f9015d audit(mpcc): prove recursive terminal successors`
- Dynamic run: `output/20260830-043824`, Domain 1
- Frozen failure: decision 3149, Follow intent, source fingerprint
  `655375378250292802`
- Same-snapshot architecture result:
  - persistent/current selected branch: rejected;
  - stateless current-world left branch: rejected;
  - stateless current-world right branch: accepted by the unchanged seven-state
    SQP, exact wall/dynamic proof and exact terminal Stop proof.

## Root cause

`evaluate_rate_resolved_normal_avoidance_population()` constructs both physical
homotopies, orders the previously selected side first and returns immediately
when that first branch is certified. Only that one plan reaches the normal
Store. The alternative branch is neither solved nor retained at the
asynchronous result boundary.

When the selected branch later fails exact current-world revalidation, the
publisher has no certified current-world alternative and correctly transfers
authority to the external Emergency Stop. The Emergency is a safety response;
the first violated invariant is that a producer described as a dual
current-world population destroys a viable branch before selection can be
repeated against the current world.

## Objective

1. Evaluate Cruise/Follow negative and positive normal-avoidance branches from
   one immutable source epoch using their existing independent solver contexts.
2. Publish both certified branch artifacts atomically as ephemeral
   current-world evidence.
3. Keep the preferred branch as the normal first choice.
4. If the chosen candidate, published Bundle source and executed plan all fail
   the existing exact current-world proof, evaluate the untried branch from the
   same atomic branch set through that unchanged proof chain.
5. Publish it only when the existing production authority and recursive Stop
   certificate succeed.
6. Delete the preferred-first early-return edge which prevented the other
   branch from being evaluated.

## Constraints

- Scope is Cruise/Follow normal dynamic avoidance only.
- Do not change committed Overtake side, no-return behavior, ShiftOut, Pass,
  Return or Recovery.
- Do not change weights, solver tolerances, horizon, wall/vehicle clearance,
  acceleration, braking or steering limits.
- Do not add a lease, grace period, timeout, retry, resume rule, feature flag or
  another normal fallback.
- The branch bank is not an execution ledger and cannot publish. Every use must
  pass the existing current-world wall, obstacle, reachability and terminal
  Stop proof.
- A newer source epoch atomically replaces both older branches, including when
  neither branch was certified. Branches from different epochs may never be
  mixed.
- The external Emergency supervisor remains the terminal outcome when no
  branch is certified.
- Generated output, rosbag and snapshot artifacts stay untracked.

## Acceptance

- Deterministic tests prove atomic dual publication, same-epoch identity,
  monotonic replacement, invalid-plan rejection and empty replacement.
- Source-contract tests prove both branches are evaluated and the old early
  return is gone.
- Full package tests and `make autoware-build` pass.
- Dynamic evidence shows an alternate current-world branch is attempted only
  after the ordinarily selected evidence fails, with its plan sequence, side
  and exact proof result traceable.
- No stale fingerprint, mixed epoch, Overtake side reversal or uncertified
  normal command is published.

# Validation

## Static gates

- `make autoware-build`: PASS, 25 packages.
- `colcon test --packages-select multi_purpose_mpc_ros`: PASS, 52/52 CTest
  targets.
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros
  --verbose`: 2027 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: PASS.

The deterministic tests prove:

- an unpublished candidate is evaluated at cursor zero;
- a published plan advances from its first actual publication control origin;
- a missing published origin is a typed execution-clock failure;
- the store records plan and origin atomically;
- a later publication of the same immutable plan does not restart its clock;
- candidate/executed ownership uses complete artifact identity rather than a
  producer-local sequence number.

## Dynamic gate

Bounded `make dev2` run: `output/20260828-202620`.

Domain 1 changed from the frozen permanent
`steering-unreachable -> retained-proof-unavailable -> Emergency` loop to:

1. a fresh candidate accepted with
   `cursor=0.000000/clock=unpublished-candidate`;
2. exact serialized actuation join with `authority=production, selected=1`;
3. subsequent retained evaluation with
   `clock=published-plan/first_publish=<finite>`.

The normal production join reported repeated successful publication windows,
including `joined=42, rejected=0`.  This satisfies the bounded gate: the cold
start no longer treats async certificate age as fictitious executed steering.

## Separate observations, not changes in this slice

The same run later entered the pre-existing Stuck Recovery path.  During that
authority interval the normal successor counters correctly showed
`submitted=0/rejected=81`; this was not a new async replenishment failure.
Recovery, its triggering physical event, and overtake completion quality are
separate frozen failures and were not patched or tuned here.

No solver tolerance, wall clearance, lease, grace period, timeout, fallback or
production-authority rule changed.

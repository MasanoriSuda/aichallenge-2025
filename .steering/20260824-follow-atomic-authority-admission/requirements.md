# Requirements

## Objective

Make entry into canonical Follow authority atomic: the requested behavior may
not leave a previously certified normal intent and enter Follow production
until a current Follow plan is solved, physically certified, and reachable
from the steering command actually published in the preceding cycle.

## Failure-first evidence

Run `output/20260824-215632`, Domain 1:

- decision 1464 published a certified Cruise command at `-0.32 rad`;
- the behavior changed `Cruise -> Follow` before a new Follow worker result
  existed;
- decision 1469 therefore inspected old Follow plan 1462 at stage 4;
- that old plan requested `-0.19 rad` from current `-0.35 rad`, outside the
  one-cycle reachable interval;
- the continuity contract correctly rejected it and exclusive Follow
  ownership had no producer, so Emergency braking was the visible result;
- decision 1470 received a new Follow plan and resumed normally one cycle
  later.

The defect is authority admission ordering, not a steering-rate parameter and
not a reason to accept the stale plan.

The first validation run, `output/20260824-221751`, exposed the same missing
admission invariant one level earlier: Behavior reported Follow while
`front_distance=inf` and no coherent front observation existed.  That label
cannot request Follow production.  It must remain Track/Cruise until current
front evidence and matching target provenance exist.

## Constraints

- Do not weaken steering continuity, target, wall, or solver certificates.
- Do not add a legacy MPC producer, command clamp, timeout, lease, grace, or
  configuration flag.
- Use the same five-state Follow problem, solver, canonical plan extraction,
  current-world proof, and final publisher as steady-state Follow.
- A transition admission solve may run only when Follow is requested, no
  canonical Follow selection is available, and the last successfully
  published canonical normal intent was not Follow.
- Serialize transition admission with the existing asynchronous Follow solver
  context; the same solver workspace may not be entered concurrently.
- Once Follow has published successfully, a later same-intent loss remains a
  fail-closed error and must not invoke transition admission.
- Keep `aichallenge/result-summary.json` untouched and uncommitted.

## Definition of Done

- The authority policy distinguishes requested-intent admission from a
  same-intent runtime failure.
- First-cycle Follow admission produces a current certified Follow command or
  fails closed; it never publishes an old unreachable Follow plan.
- The transition solve reuses the canonical Follow producer rather than a new
  formulation or downstream fallback.
- Focused tests, all package tests, and `make autoware-build` pass.
- A bounded `make dev2` no longer emits an async-pending Emergency solely on
  `Cruise -> Follow` or initial Follow entry.
- A Follow behavior label without coherent front evidence resolves to
  Track/Cruise production, not Follow or Emergency.

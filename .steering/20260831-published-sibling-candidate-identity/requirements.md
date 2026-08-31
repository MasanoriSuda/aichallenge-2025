# Requirements: published sibling candidate identity

## Objective

Audit and remove the authority discontinuity observed immediately after a
stateless sibling Bundle crossed the canonical publisher in
`output/20260831-160811/d1/autoware.log`.

## Frozen observation

- decision 1205 publishes sequence 589 and atomically changes the encounter
  side from `-1` to `+1`;
- the next alignment initially sees `current-world-bundle/source_side=+1`;
- a retained candidate from the pre-adoption side is then accepted as ordinary
  current-world authority;
- the latest published source for the same artifact sequence becomes side
  `-1`, while tactical state remains side `+1`;
- execution-source projection reports `side-mismatch`, followed by stale or
  missing authority and Stop/Recovery.

## Constraints

- no Mission resume rule, lease, grace period, timeout or fallback;
- no wall clearance, solver tolerance or parameter change;
- do not weaken current-world, physical wall, dynamic obstacle or terminal
  successor proof;
- do not change target, homotopy or no-return policy;
- a plan from the non-live side may obtain authority only through the existing
  publisher-bound sibling adoption token.

## Definition of done

- direct candidate/published/executed retained evaluation cannot republish an
  Overtake plan whose target, generation or side disagrees with live tactical
  identity;
- the dedicated sibling resolution path remains the only cross-side authority
  path;
- focused tests and package build pass;
- dynamic evidence shows sibling adoption without immediate source-side
  rollback or published alignment `side-mismatch`.

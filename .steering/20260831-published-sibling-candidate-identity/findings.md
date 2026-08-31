# Findings: published sibling candidate identity

## Observed failure

After a publisher-bound sibling token changed an active ShiftOut from side
`+1` to side `-1`, a candidate captured before the adoption could still enter
the generic retained-authority path.  Physical revalidation did not reject it,
so the old side could overwrite the newly published tactical identity.

## Root cause

The generic candidate, published-Bundle and executed retained paths treated
current-world geometric proof as sufficient authority.  They did not also
require the Overtake plan's immutable `{intent,target,generation,side}` to
match the live tactical encounter.  The dedicated sibling path had that check,
but the generic path was an unintended second cross-side authority path.

## Change

- capture the live active-Overtake identity once per retained evaluation;
- evaluate candidate, published-Bundle and executed plans against that identity;
- keep opposite-side inspection in the dedicated sibling resolver only;
- expose each identity decision in the retained summary log;
- reject old-side in-flight work without adding a lease, grace period, timeout,
  fallback or parameter change.

## Validation

- focused source contract: `97 passed`;
- `make autoware-build`: 25 packages successful;
- dynamic acceptance: `output/20260831-162051/d1/autoware.log`.

Two sibling adoptions occurred.  In both cases the current-world Bundle was
published on the new side.  No `Published Overtake execution alignment` rolled
back with `side-mismatch`.  The old direct candidate was explicitly observed as
`tactical_identity=candidate:1:side-mismatch` and did not reclaim authority.

## Newly exposed boundary

Immediately after each valid adoption, the six-state execution-source
projection still rejected the old solved trajectory cache with
`reason=side-mismatch`.  The first episode then entered Recovery with
`reason=live overtake corridor unavailable`; the second later lost its plan.

That is a separate execution-cache handoff defect.  The publisher identity is
now correct, but the exact newly published trajectory is not atomically made
the live six-state execution source.  It must be investigated in the next
Slice rather than hidden by weakening the identity guard.

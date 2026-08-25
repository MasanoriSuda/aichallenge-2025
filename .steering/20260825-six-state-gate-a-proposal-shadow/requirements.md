# Requirements

## Objective

Prove that the selected six-state pre-entry result can reach the actual
Overtake FSM Gate-A boundary as one typed, current-world-valid proposal before
any Mission state mutates.

## Root cause being tested

The causal execution worker currently finishes after tactical evaluation, but
its result is consumed inside normal control after `update_overtake_line()` has
already admitted or rejected entry through the five-state Gate A. A complete
six-state proof therefore cannot replace that gate without first correcting
the update order.

## Scope

- carry the exact selected Mission hint with the causal six-state result;
- consume the result before `update_overtake_line()`;
- expose Mission, CertifiedPlan and immutable identities as one shadow-only
  Gate-A proposal;
- repeat current-world proof at that actual boundary;
- record whether the proposal would be ready for atomic admission.

## Non-goals

- no production authority change;
- no Mission/FSM mutation from the proposal;
- no certified-plan store mutation or command publication;
- no new fallback, flag, timeout, lease, parameter or solver setting;
- no removal of the five-state Gate A in this evidence Slice.

## Invariants

- a proposal is complete only when Mission side, six-state execution side,
  target, prospective generation and tactical identity agree;
- stale physical trajectories still fail current-world revalidation;
- the proposal is created before `update_overtake_line()` and is not consumed
  by that function in this Slice;
- async context invalidation removes any retained proposal.

## Definition of Done

- source contract proves the ordering and shadow-only isolation;
- build and package tests pass;
- bounded `make dev2` shows at least one complete proposal at the FSM boundary,
  or gives a typed reason why it cannot be formed;
- callback telemetry has no new overrun regression;
- evidence identifies the exact production promotion/deletion boundary.

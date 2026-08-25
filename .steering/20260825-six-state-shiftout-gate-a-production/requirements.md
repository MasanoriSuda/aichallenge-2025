# Requirements

## Objective

Promote only the dynamically observed fresh `ShiftOut` Gate A from the
five-state pre-entry proof to the causal six-state atomic proposal.

## Root cause

Normal execution is already owned by the six-state producer, but fresh
ShiftOut admission still freezes a Mission only after a five-state physical
certificate and five-state canonical plan have passed Gate A. This leaves a
second formulation controlling whether the six-state owner may start.

## Scope

- admit fresh ShiftOut only from the current-cycle typed six-state proposal;
- freeze supervisor Mission geometry from that same proposal;
- require exact target, side, generation and ShiftOut intent identity;
- retain the existing post-transition six-state solve, physical proof and
  current-world join before normal publication;
- remove the five-state certificate/canonical-plan requirement from the
  ShiftOut branch in the same Slice;
- keep direct Pass unchanged because it has no dynamic proposal coverage yet.

## Non-goals

- no direct-Pass promotion;
- no active-Mission same/cross-side replan migration;
- no parameter, solver, wall-margin or timing adjustment;
- no new fallback, flag, timeout or lease.

## Definition of Done

- source contracts fail if ShiftOut can use five-state Gate A or if a proposal
  mismatch mutates Mission state;
- build and full package tests pass;
- bounded `make dev2` records a six-state ShiftOut Gate-A admission followed
  by six-state normal publication, or a typed fail-closed reason;
- five-state entry remains only in the explicitly unpromoted direct-Pass path;
- callback overrun and Emergency regression are reported before acceptance.

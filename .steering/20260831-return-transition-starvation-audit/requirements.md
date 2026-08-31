# Requirements: Return transition starvation audit

## Problem

In `output/20260831-134900/d1`, Return Gate A drafts were submitted every
control cycle from decision 1535 through decision 1563.  No Return proposal
became available, so the controller retained a certified Pass for about
0.7 seconds.  At decision 1563 the retained Pass and its current-world Stop
successor both became infeasible.

## Objective

Identify whether Return authority is lost in worker execution, latest-only
publication, current-world consumption, or transition identity admission.

## Constraints

- Observation only in this Slice.
- No production authority, timeout, lease, fallback, solver, wall or
  clearance change.
- Preserve immutable decision, intent generation and tactical identity.

## Definition of Done

- Return deferral reports worker submitted/replaced/started/completed and
  queue state.
- Return completion reports sequence, source decision, compute time, solver
  outcome and whether latest-only publication accepted it.
- A bounded run classifies the first missing Return proposal.
- Classification is written before any production fix.


# Design

## Root cause and remaining migration debt

The prospective pre-entry worker already derives `Pass` when the selected
Mission is a fresh direct pass and derives `ShiftOut` otherwise.  It solves,
physically certifies and current-world joins an exact six-state artifact for
either intent.

The FSM boundary nevertheless recognizes only a six-state `ShiftOut` proposal.
Direct Pass falls through to `overtake_preentry_canonical_plan` and
`resolve_overtake_preentry_plan()`, the retained five-state Gate A.  Normal
publication is six-state after commit, but Mission adoption is still decided
by a different formulation.  This is the last reachable cross-formulation
normal-entry authority in Slice 6.

## Selected repair

1. Treat the prospective proposal as a generic six-state pre-entry proposal
   whose intent must be exactly `ShiftOut` or `Pass`.
2. Derive direct-pass entry from the proposal intent and its immutable Mission,
   not from a separate five-state plan.
3. Require the proposal intent to match the FSM entry phase before mutation.
4. Commit the proposal Mission atomically, then let the shared six-state normal
   production path solve and adopt the effective intent.
5. Remove the five-state resolver and canonical-plan requirement from the
   production entry path in the same change.
6. Remove the pre-entry-only manual predecessor binding.  Use the shared binder
   so physical steering at the control origin and the exactly published
   steering are both sealed before snapshot construction.

The five-state tactical branch may remain diagnostic/reference generation in
this Slice; it cannot admit a Mission or publish a normal command.

## Rejected alternatives

- Keep five-state Direct Pass as fallback: preserves two formulations at one
  authority boundary.
- Convert a five-state artifact to six-state identity: fabricates steering
  state/rate provenance.
- Admit Direct Pass from geometry only: mutates Mission before Gate A.
- Tune direct-pass thresholds: does not fix formulation ownership.

# Design: Pre-entry semantic identity

## Root cause

`build_prospective_extended_branch_problem()` temporarily installs a selected
Overtake candidate, then calls `init_problem()` without its semantic intent.
`init_problem()` therefore derives profile ownership from the live controller
intent, while the later seven-state builder is explicitly called with
ShiftOut or Pass.  This creates one candidate with two semantic owners.

For the frozen failure, the source skipped the progress-indexed physical wall
profile, but the seven-state adapter still accepted the prospective ShiftOut
scope.  It consequently imposed a future stage-indexed lateral wall box before
progress had moved, making the first dynamics equality and lateral box
mutually infeasible.

## Repair

Introduce one pure pre-entry intent resolver in the MPCC execution contract.
Compute the intent before `init_problem()` and pass it through the existing
`semantic_intent_override` argument.  Reuse the identical result when building
the extended problem.

This restores the intended producer chain:

```text
candidate + source phase
  -> one prospective intent
  -> init_problem(intent)
  -> wall/obstacle/profile semantics
  -> build_extended_progress_problem(same intent)
  -> immutable problem fingerprint
```

No downstream clamp is added.  The existing pre-refinement union support and
progress-aligned physical wall refinement remain the owners of the initial
convexification.

## Alternatives rejected

- Widen stage-one lateral bounds: hides a coordinate/ownership defect and may
  relax opponent-owned constraints.
- Increase solver iterations: the frozen short QP is linearly infeasible.
- Reject all zero-speed candidates: removes a useful stopped-vehicle passing
  case rather than repairing its formulation.

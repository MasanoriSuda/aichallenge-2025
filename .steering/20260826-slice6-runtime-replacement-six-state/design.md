# Design

## Causal fault

The tactical dual branch already evaluates active ShiftOut/Pass phases with a
prospective Mission generation and produces an exact six-state solver, wall and
current-target proof.  The causal worker joins that proof to the current world
and emits a Mission Gate A proposal.

Runtime Mission replacement does not consume that proposal.  Two MPCC-lite
paths carry `OvertakeExecutionArtifact`, whose immutable execution plan is a
five-state artifact.  Other runtime wall, dynamic-wait, opponent-side and
safe-separation paths carry only a geometric Mission.  All eventually call
`replace_frozen_overtake_mission_after_dynamic_replan()`, which can mutate the
Mission before the next six-state normal solve.

The observed failure may therefore surface later as solver rejection or
authority loss, while its source is an unproved Mission mutation.

## Selected repair

1. Treat the causal six-state proposal as a generic Mission Gate A, valid for
   fresh entry and runtime replacement.
2. Make every runtime replacement request provide the current proposal.
3. Before any fallible state mutation, require exact target, target generation,
   requested side, prospective Mission generation and derived execution intent.
4. Run replacement physical/budget contracts against the proposal's immutable
   Mission, never a separately reconstructed candidate.
5. On rejection, keep the currently proven Mission and record the causal
   identity mismatch.
6. Delete `OvertakeExecutionArtifact` and
   `resolve_overtake_preentry_plan()` after the six-state boundary owns all
   runtime replacement.

## Rejected alternatives

- Keep five-state runtime replacement as fallback: preserves two formulations
  at one authority boundary.
- Convert a five-state plan into a six-state identity: fabricates steering-rate
  provenance.
- Allow geometry-only replacement then solve six-state next cycle: state has
  already mutated before proof.
- Disable all runtime replan: structurally safe but removes intended dynamic
  behavior despite an existing causal proof pipeline.

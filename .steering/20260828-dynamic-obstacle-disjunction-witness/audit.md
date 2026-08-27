# Audit

## Frozen evidence

- baseline: `e098f994`
- run: `output/20260828-034416`
- snapshot: sequence 2187, Pass dynamic-obstacle refinement
- exact production QP: infeasible

The replay command was:

```text
python3 replay_disjunction.py <sequence-2187-snapshot.yaml>
```

Result:

```text
old_disjunction_feasible=0
first_witness_transition_stage=1
full_separation_transition_feasible_stages=
initial_separation_nonworsening_feasible=1
witness_preserving_disjunction_feasible=1
```

This does **not** prove that a safe late escape exists.  It proves only that
the production stay-behind disjunction caused the algebraic infeasibility.
No late full-separation transition is feasible in the frozen linearization,
so accepting the weak partial witness would exchange an explicit QP failure
for an uncertified collision trajectory.

## Failure-first regression

The new test fixes an explicit negative tactical side whose stage-zero state
already has full lateral separation while its obstacle-free wall witness
crosses back later.  Before the repair the refiner emitted stay-behind rows;
after the repair it emits full selected-side rows for every stage.  The
existing `DoesNotTrustOneSeparatedMiddleSample` test remains green.

## Verification

- `make autoware-build`: pass
- package CTest: 49/49 pass
- solver, clearance, timeout, lease, fallback and authority: unchanged

## Dynamic acceptance

Bounded `make dev2` Gate: `output/20260828-041315`.

The run did not reach the repaired condition:

- Episode 1 reached `Idle -> ShiftOut`;
- lateral separation became true, but no `ShiftOut -> Pass` occurred;
- the frozen ShiftOut Mission later reported
  `physical target separation conflicts with wall bounds`;
- canonical execution became unavailable and Emergency Stop took authority;
- the episode ended with `actual footprint wall margin violated`.

Therefore the acquired-Pass invariant is statically verified but dynamically
unexercised.  The next earliest defect is now clearer: a reference-complete
but non-executable persistent ShiftOut Mission can remain tactically active
after its current-world wall/target intersection becomes empty.  Snapshot
1150 is mathematically infeasible even without warm-start state.  No tuning
was performed.

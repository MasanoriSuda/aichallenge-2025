# Audit log

## Entry inventory

| Entry | Physical wall | Immutable dynamic world | Finding |
|---|---|---|---|
| Track/Cruise async submission | bound | bound | complete |
| synchronous pre-entry evaluation | bound | bound | repaired |
| asynchronous pre-entry evaluation | bound | bound | repaired |

The two pre-entry gaps were upstream integration defects. Both now bind the
same immutable `ReplayWorld` contract used by Track/Cruise before entering the
common solver. A missing world cannot authorize a physical diagonal; a present
world whose generation differs from the problem identity rejects assembly.
Neither case authorizes a weakened separating row.

Runtime evaluation also compares `ReplayWorld::observation_generation` with
the sealed problem target generation before resolving physical geometry. A
present but stale world now returns `AssemblyRejected` with an explicit
identity/provenance reason.

## Production repair gate

Open for dynamic acceptance. The automatic topology selector reproduced the
frozen certified bundle and all three production solver entries now carry the
same immutable geometry contract.

## Automatic witness-tangent falsifier

A schedule-free candidate G was evaluated before production changes.  It
formed the physical support tangent at every stage from the wall-only witness.

- left, one tangent solve: QP and wall proof passed, but exact dynamic proof
  rejected a new overlap of `0.000865 m` at `t=1.14275 s`;
- the affine/nonlinear node discrepancy reached `0.0456912 m`, with maximum
  witness-to-solution heading error `0.241273 rad`;
- a deterministic second SQP tangent iteration made the left QP infeasible;
- right remained solver-infeasible.

Therefore a single automatically rotating local tangent is not the production
repair.  Increasing SQP iterations or relaxing proof/solver tolerances is not
authorized.  Candidate G code must not remain in production.

## External and upper-rank comparison

The official T-MPC++ implementation and paper separate topology guidance from
parallel local MPC solves.  Distinct obstacle-passing homotopies are generated,
optimized independently and only then compared; a non-guided regular MPC is
also evaluated in parallel.  This directly addresses local-MPC entrapment and
does not rely on one tangent changing topology.

The upper-rank `.steering/ano` log has the same structural signature:

- one continuously running GMPCC, `N=20`, `dt=0.12`, horizon `2.4 s`;
- a separate async child process;
- ranked candidate population (`rank=.../3`);
- the main solved trajectory continues while candidate work runs.

Together with frozen candidate F, this points to a bounded, distinct
behind-to-side homotopy seed—not more iterations of candidate G.

## Derived production topology

For the exact initial-overlap case formerly masked by `partial_side_escape`,
the earliest mutable obstacle stage is one stage after the first valid
prediction.  The physical diagonal contract requires two stages between its
longitudinal and lateral endpoints.  The canonical schedule is therefore
derived as:

```text
start = first_valid_stage + 1
full_side = start + 2
```

This is relative to the current horizon/encounter and not the frozen absolute
`1 -> 3` pair.  If the horizon cannot contain it, no candidate is produced.
The frozen snapshot is the deterministic falsifier for this derivation.

## Frozen production-equivalent replay

After removing `partial_side_escape`, the complete package test suite passed
(`52/52`). Replaying decision `1566` with no candidate-F-only production flag
produced:

```text
arm=stateless-left-b
stage=accepted
candidate=14650321662952263845
solver=solved
terminal_progress=19.8243
terminal_velocity=4.62416
lateral_reserve=0.0532911
bundle=1
```

The frozen persistent arm still failed at the historical wall-refinement row,
and the right homotopy remained solver-infeasible. This is the intended
classification: rebuilding the stateless current-world bundle with the
physical disjunction succeeds, while retaining the old persistent geometry
does not. No solver, timeout, clearance or tolerance setting changed.

The accepted terminal metrics exactly match the previously certified
candidate-F evidence. The candidate fingerprint differs because this replay
uses the canonical stateless-B identity rather than the offline-F arm identity.

## Deleted legacy contract

The following production/migration surface was removed atomically:

- wall-only witness values as partially separating obstacle rows;
- `partial_escape_row_count` and its shadow/controller telemetry;
- unit tests which required partial separation to be treated as a valid row.

Without immutable physical geometry, only complete behind/side disjuncts are
emitted. With immutable geometry, the derived diagonal is emitted and remains
subject to the unchanged nonlinear wall/dynamic/terminal proofs.

## Static validation

- `make autoware-build`: 25 packages passed;
- focused dynamic-obstacle and shadow tests: 2/2 passed;
- complete `multi_purpose_mpc_ros` CTest: 52/52 passed;
- experiment registry: 3 snapshots and 19 experiments valid;
- `git diff --check`: passed.

## Dynamic Gate result

Run `output/20260828-132039` exercised the automatic physical diagonal twice,
confirming that the production binding is active. Both diagonal QPs were
solver-rejected and no diagonal artifact was published. A separate ordinary
Pass reached `ShiftOut -> Pass`, but then retained a target-bound path after
its current dynamic certificate became unsafe and ended in SafetyBrake rather
than Return. The diagonal Slice therefore remains statically accepted but
dynamically inconclusive; the downstream lifecycle defect is handled in
`20260828-target-bound-current-world-proof` without tuning this topology.

After that lifecycle repair, run `output/20260828-133920` produced an ordinary
`ShiftOut -> Pass -> Return` episode. This is evidence that Pass/Return remains
reachable after deleting unsafe retained geometry, but it does not accept the
physical-diagonal production path: no diagonal artifact was published in that
episode, and Return later lost canonical authority and required external
Recovery. Physical-diagonal dynamic acceptance therefore remains open.

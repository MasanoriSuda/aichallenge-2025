# Design: Return-entry target contract audit

## Initial observation

At decision 1688, sequence 936 still published a certified Pass command. Return
drafts were submitted repeatedly, but the transition worker reported
`canonical current-epoch target tube unavailable`. At decision 1690 the Pass
terminal Stop successor became unavailable, normal authority fell to Emergency
Stop at 5.35 m/s, and the tactical FSM then moved to FollowPrepare and wall
Recovery.

## Competing hypotheses

1. Persistent lifecycle defect: a stateless current-world Return can certify
   without depending on the stale Pass artifact.
2. Candidate-generation defect: current Return construction incorrectly makes
   a full target tube mandatory after the target relation is already terminal.
3. Single-SQP limitation: Return geometry exists but needs another
   linearization/topology.
4. Physical infeasibility: no exact wall-safe Return or Stop successor exists at
   the frozen state.
5. Model/certificate mismatch: solve succeeds but terminal/wall proof rejects a
   semantically valid trajectory.

No hypothesis is accepted before same-snapshot comparison.

## Earliest-invariant result

Decision 1690 is too late for repair: persistent, stateless, rough and offline
arms all fail from that already-diverged state. Sequence 942 changes the
classification. On its immutable world the unchanged bounded production
population certifies both sides, while live evaluation reported both sides
uncertified after an accepted exact wall proof:

```text
exact_dynamic_final=invalid/blocked/obstacle=
```

The invalid result occurs before an obstacle is checked. `ReplayWorld` captures
the physical snapshot's baseline `bound_tolerance_m=1e-5` before solve. The
accepted artifact then owns a measured-residual tolerance (about `4.16e-5` for
the frozen family), and the final wall proof correctly uses it. Dynamic proof
incorrectly required those pre- and post-solve values to be identical.

The repair assigns responsibilities as follows:

- ReplayWorld owns immutable world geometry and observation provenance;
- the exact trajectory and final wall snapshot jointly own the post-solve
  lateral certificate tolerance;
- dynamic proof verifies that certificate pair, not a pre-solve placeholder;
- every source-validation rejection gets a structured reason in telemetry.

No Mission rule, authority, solver setting or physical margin changes.

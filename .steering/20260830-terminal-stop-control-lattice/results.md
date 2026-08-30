# Results

## Frozen comparison

The bounded lattice uses horizon-relative switch grids:

- first switch: 10%, 15%, 20%, 25%, 30%;
- second switch: 30% through 60% in 5% increments;
- initial steering-rate sign: both positive and negative.

Every future velocity state is fixed to the solver-safe maximum-braking
schedule.  The canonical adapter builds the normal seven-state problem before
the audit solver fixes steering-rate input rows.  The exact nonlinear physical,
wall and current-world proofs are unchanged.

### ShiftOut side positive, decision 4017

| Arm | Result | Solve | Certified control |
|---|---|---:|---|
| free seven-state Stop | accepted | 157.54 ms | free steering-rate sequence |
| bounded control lattice | accepted | 54.53 ms | positive 3 stages, negative 3 stages, hold |

The lattice stopped at local progress `5.49297 m`, terminal velocity
approximately zero and exact minimum lateral reserve `0.374927 m`.  It was the
eighth evaluated candidate.  The SQP output preserved the requested
positive/negative/hold sequence; only numerical residuals below `1e-9 rad/s`
remain in the hold interval.

### Pass side negative, decision 4489

| Arm | Result | Solve | Certified control |
|---|---|---:|---|
| free seven-state Stop | accepted | 88.32 ms | free steering-rate sequence |
| bounded control lattice | accepted | 48.83 ms | positive 3 stages, negative 3 stages, hold |

The lattice stopped at local progress `4.00106 m`, terminal velocity
approximately zero and exact minimum lateral reserve `0.0693029 m`.  It was
also the eighth evaluated candidate.

## Horizontal replay boundary

Seven other historical terminal-contingency snapshots were sampled.  One old
v3 snapshot was incomplete.  In five cases the source normal SQP could not be
reproduced from a cold offline context, so neither the free Stop nor lattice
Stop could be rebased at the publisher boundary.  One Pass snapshot reproduced
both arms and is reported above.

Those cold-source failures are not evidence against the lattice.  They are a
separate scheduling/warm-start observability gap: the live failure occurred
after a normal artifact existed, but the failure snapshot did not serialize
that artifact or exact QP.  Production integration must not be inferred from
those snapshots until that evidence boundary is repaired.

## Root-cause classification

For both reproducible failures:

```text
fixed/path-feedback maximum-braking Stop fails
bounded bang-bang lattice + same single seven-state SQP succeeds
free seven-state Stop succeeds
```

This is a candidate-generation defect.  It is not physical infeasibility, a
single-SQP limitation or a need to weaken wall/clearance constraints.

The accepted lattice is substantially simpler than the free control sequence
and solves faster per candidate.  Production should therefore gain one bounded
certified Stop candidate source within the existing asynchronous computation
boundary.  It should replace the fixed-target/path-feedback candidate when
promoted, not become another permanent fallback chain.  Authority remains
unchanged in this Slice.

## Verification

- focused architecture-comparison test: passed
- fixed steering-rate audit contract test: passed
- full package CTest: 59 / 59 passed

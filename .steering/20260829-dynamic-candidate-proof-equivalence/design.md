# Design: dynamic candidate/proof equivalence

## Hypotheses

### H1: ordinary axis rows do not represent the exact oriented footprint

The ordinary side/behind rows use scalar `StagePrediction` separations.  The
exact certificate uses an asymmetric rectangle with heading plus a moving
circle.  A row can therefore be feasible while an ego corner overlaps the
circle.  This is supported if affine stage-node clearance is negative while
the scalar disjunction reserve is nonnegative.

### H2: stage nodes are physically clear but the swept segment is not

The QP constrains stage states; the certificate subdivides each moving segment.
This is supported only when all affine and nonlinear stage nodes are clear but
the first rejected time lies strictly between nodes.

### H3: affine dynamics and exact integration diverge

This is supported when affine stage-node clearance is clear but the
corresponding nonlinear stage-node clearance is not, with a material node
position error.

### H4: the physical problem is infeasible

This is supported only if persistent, stateless, lattice/physical candidates,
and bounded offline nonlinear or multi-SQP feasibility attempts all fail the
same unchanged physical proof.

## Investigation order

1. Reuse existing A--G architecture comparison for representative snapshots.
2. Inspect and report ordinary row geometry versus exact footprint support.
3. Add observation-only diagnostics if the recorded values are insufficient.
4. Test a proof-consistent node-row formulation on the frozen corpus.
5. Address swept-segment coverage only if node equivalence is established and
   inter-stage-only failures remain.

## Intended replacement boundary

`StagePrediction` remains the obstacle time/progress/lateral prediction.  The
selected dynamic disjunct must use the same immutable physical geometry and
heading witness as the exact certificate.  Production must not retain two
competing definitions of required body separation.

The exact certificate remains the sole publication proof.  The QP model is
made consistent with it; the certificate is not weakened to accept the QP.

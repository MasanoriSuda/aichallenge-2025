# Design

## Offline restoration arm

```text
unchanged broad seven-state solve
  -> unchanged first physical wall refinement
  -> affine infeasible
  -> audit-only heading-box Phase-I projection QP (not certifiable)
  -> bounded SQP tangent updates inside the wall-directed seed problem
  -> rebuild fresh physical wall buckets from the final seed
  -> solve unchanged full seven-state QP
  -> unchanged exact trajectory/wall/dynamic/successor proofs
```

The selective LP audit showed that removing only refinement-owned heading
boxes restores affine feasibility for decision 2473. Therefore the first arm
relaxes exactly that family back to the pre-refinement semantic bounds. It
does not relax lateral, lag, progress, steering, wall rows, inputs, or the
initial state.

The Phase-I objective is a strictly convex projection toward the preceding
seed. The constraint matrix and every physical/actuation bound are unchanged;
the racing-performance objective is deliberately absent because this arm asks
only whether a feasible tangent exists.

The relaxed QP is deliberately non-authoritative because its wall interval
was certified around the preceding heading bucket. Its only output is a
numerical tangent. Fresh physical wall buckets and the complete QP are rebuilt
afterward; only that new problem may create an execution artifact.

## Classification

- final full QP and proofs succeed: current one-shot wall refinement is the
  root defect and the restoration construction is a viable replacement;
- restoration seed succeeds but final full QP stays affine-infeasible:
  heading-only restoration is insufficient; proceed to nonlinear/multi-SQP
  feasibility, not parameter tuning;
- final solve succeeds but exact proof fails: affine wall model/certificate
  mismatch;
- restoration seed itself is infeasible: revisit row-family diagnosis before
  expanding the restoration scope.

## Independent backend proof

If the restored full QP is exactly feasible but the production OSQP backend
does not converge, arm I accepts an externally solved primal only as audit
data.  It first rechecks every immutable recorded QP row with the production
physical tolerance and then rebuilds the same execution artifact, nonlinear
trajectory, wall proof, all-obstacle dynamic proof and terminal successor.
It has no solver lifecycle, command conversion, store, mailbox or publisher.

This separates two materially different outcomes:

- external primal fails an exact downstream proof: model/certificate mismatch;
- external primal passes all proofs: backend/convergence mismatch after the
  feasibility construction defect is repaired.

## External design check

The ETH Zurich MPCC reference implementation constructs track half-spaces and
an obstacle corridor before solving its time-varying QP. Current acados SQP
implementations expose globalization and feasible-QP strategies for cases in
which a local QP does not provide a usable step. These are architecture
references only; no new dependency is introduced in this Slice.

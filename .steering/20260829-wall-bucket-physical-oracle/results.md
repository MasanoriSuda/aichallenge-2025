# Results: wall-bucket physical oracle

## Frozen evidence

- run: `output/20260829-122448`
- decision: `639`, intent `ShiftOut`
- source fingerprint: `11478535197026802675`
- failure: `wall-refinement-solve-rejected`

Independent Phase-I analysis found the full recorded affine QP infeasible.
Removing either the lag or heading wall-pose bucket made the QP affine
feasible.  HiGHS, qpOASES, ProxQP and explicitly scaled OSQP agreed on those
relaxed problems.

## Exact proof of the bucket solutions

The independently solved one-bucket primals were passed through the unchanged
C++ execution proof chain:

- omit lag: exact trajectory exceeded the upper lateral corridor by about
  `0.00188 m` near dense sample 301;
- omit heading: exact trajectory exceeded it by about `0.00524 m` near dense
  sample 300.

Therefore deleting either bucket is not a valid production fix.

## Nonlinear physical oracle

Starting from the HiGHS omit-lag control sequence, the exact seven-state
control-only oracle converged in 12 L-BFGS-B iterations.  Its minimum retained
physical constraint margin changed from `-0.0149988 m` to approximately
`-4.76e-9 m`.

The complete external primal was then checked by the normal C++ proof chain:

- execution artifact: accepted;
- exact nonlinear trajectory: accepted;
- physical wall sweep: accepted;
- current-world dynamic obstacles: accepted;
- terminal successor and Stop suffix: accepted;
- minimum exact lateral reserve: `0.0152999 m`;
- terminal progress: `14.825 m`;
- terminal velocity: `6.49899 m/s`.

The observation-only arm produced a complete `ManeuverBundle`.  It has no
Store, mailbox, command or publisher access.

## Root classification

The maneuver is not physically infeasible.  The current hard post-hoc
lag/heading buckets and one-shot affine dynamics exclude a physically
certifiable continuation.  This is a **single-SQP / model-certificate
mismatch**, not a clearance, timeout, Mission-resume or generic OSQP-tolerance
problem.

The feasible nonlinear trajectory's final lag is about `-0.5222 m`, while the
recorded stage-20 lag bucket is `[-0.275, -0.225] m`.  One frozen bucket cannot
represent the necessary progress-state reconciliation.

## Production direction

Do not remove wall buckets and do not publish the relaxed QP.  The structural
replacement must manage the trust region as part of a proof-guided/globalized
successive-convexification loop:

1. generate a feasibility tangent when the first hard bucket is empty;
2. rebuild dynamics and physical wall evidence around one common iterate;
3. evaluate each iterate with the exact proof chain;
4. keep the first/last certified artifact rather than the last numerical
   iterate;
5. leave the last actually published certified command authoritative when no
   new iterate certifies.

This is consistent with the trust-region/virtual-control safeguards described
by [Mao, Szmuk and Acikmese](https://arxiv.org/abs/1608.05133), the
globalization contracts exposed by [acados](https://docs.acados.org/), and the
separation of obstacle-side corridor selection from MPCC refinement in the
[ETH MPCC reference](https://github.com/alexliniger/MPCC).

## Verification

- `make autoware-build`: 25 packages passed;
- focused CTest run: 2,085 tests, zero failures;
- physical-oracle negative test proves bypassing affine residual checks cannot
  bypass the exact trajectory/proof chain.

Production authority and all runtime configuration values remained unchanged.

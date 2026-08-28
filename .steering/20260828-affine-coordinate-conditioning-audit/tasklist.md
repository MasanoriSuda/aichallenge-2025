# Tasklist

- [x] Freeze the two remaining affine-feasible snapshots.
- [x] Reproduce scale-only convergence.
- [x] Evaluate affine-centred convergence at the unchanged 4,000 iterations.
- [x] Compare independent active-set/proximal QP backends.
- [x] Compare exact equality-condensed OSQP formulation.
- [x] Decide whether a production implementation is justified.
- [x] Record results and commit the audit-only slice.

No production implementation is justified by this Slice. Affine centring and
equality condensing both falsified their narrow hypotheses. A solver-backend
change requires a separate architecture/dependency/real-time Slice.

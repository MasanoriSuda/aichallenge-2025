# Design

## Root cause

`build_follow_longitudinal_contract()` correctly treats a low policy speed as
a target rather than an instantaneous state constraint.  It raises the hard
velocity cap to the speed reachable under maximum braking.  Its caller,
however, supplies `abs(cfg.a_min)`.

Later, `mpcc_rate_resolved_adapter::build()` insets the acceleration input box
by the maximum accepted solver residual.  With the current immutable solver
tolerance this changes `[-3.0, 1.37]` to approximately
`[-2.959596, 1.329596]`.  The Follow state cap continues to descend at
`3.0 m/s^2`; the optimized state can descend only at `2.959596 m/s^2`.
After several stages these boxes contradict the exact velocity dynamics.

The first dynamic Gate exposed the second half of the same ownership defect.
The contract used `current_speed_mps_`, but canonical state zero used
`control_origin_speed_mps_` after measured-to-control delay prediction.  At
snapshot 1364 the QP started about 0.047 m/s faster than the envelope origin,
making stage one unreachable even with the corrected acceleration lower bound.

## Selected repair

Promote the existing exact-boundary inset calculation to a public pure
contract in the rate-resolved adapter.  The adapter continues to call it when
building input boxes.  `SolverContext` exposes its immutable physical
constraint tolerance, allowing the live Follow contract builder to resolve
the exact same acceleration interval before constructing its reachable speed
envelope.

The contract also uses `control_origin_speed_mps_`, which is the velocity
stored in canonical state zero.  Measured speed remains available to the
current-world retained proof but does not define a future control-origin
horizon.

The tactical Follow policy is otherwise unchanged.  There is no new margin or
fallback: the semantic layer simply stops promising stronger braking than the
canonical executable QP permits.

## Rejected alternatives

- Relax the frozen velocity caps after assembly: hides the ownership mismatch
  inside a downstream repair.
- Change OSQP tolerance or accept inaccurate failure: cannot repair an empty
  affine feasible set.
- Keep Cruise longer across the transition: masks the contradictory Follow
  problem and creates another lifecycle rule.
- Raise all velocity caps by a fixed epsilon: a tuning patch with the wrong
  units and horizon dependence.

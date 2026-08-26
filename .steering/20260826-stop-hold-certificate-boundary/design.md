# Design

## Observed causal chain

```text
stationary semantic horizon contains v_theta bound [0, 0]
  -> adapter preserves the valid singleton instead of making it empty
  -> OSQP returns a row-certified residual near zero
  -> immutable artifact validator compares v_theta to [0, 0] with zero tolerance
  -> every solved artifact becomes invalid-control-stage
  -> no physical wall proof or executed plan can exist
  -> canonical production publishes Emergency Stop forever
```

The visible control-mode reassertion and launch timeout are consequences of
the zero command.  They are not the first fault.  The solver reports a valid
solution and the steering sequence is sampled successfully before artifact
validation rejects it.

## Root cause

Commit `83edbba` correctly distinguished predicted velocity and virtual
progress from physical publisher inputs and preserved their zero-width
stop/hold bounds.  `mpcc_rate_resolved_execution_artifact::validate()` still
contains the earlier exact comparison introduced with production promotion:
even a certified numerical residual below the solver's physical tolerance is
rejected.  Producer and consumer therefore use different row contracts.

## Selected repair

Use the artifact's already-sealed `physical_global_tolerance` for the virtual
progress bound comparison, as is already done for lateral state and steering
limits.  Keep the semantic lower bound itself nonnegative and retain the
separate progress-dynamics and certified-regression checks.

Acceleration remains exact against its original physical envelope because the
solver bounds are deliberately inset before solve.  Steering rate likewise
keeps its physical-limit check.  No published command limit is relaxed.

The moving replay exposed the same partial propagation one boundary later:
the immutable artifact accepts a solver-certified predicted-velocity residual,
but `ExactPhysicalExecutionTrajectory` previously carried no velocity-bound
certificate and therefore re-imposed exact `velocity >= 0`.  Add an explicit
lower-bound tolerance to that immutable trajectory, seal it from the artifact's
measured maximum constraint violation, and validate raw states against it.  Do
not clamp or rewrite the solved velocity.

## Removed mask / obsolete assumption

Replace the test assertion that any `-1e-9` virtual progress must be rejected.
The corrected contract rejects only values beyond the sealed physical
tolerance and separately proves progress dynamics/regression.

## Alternatives rejected

- Clamp solver output to zero: hides a producer/consumer contract mismatch and
  mutates the certified solution.
- Re-introduce an inset for `[0, 0]`: recreates an empty QP interval.
- Use Emergency or legacy MPC at launch: adds another normal authority and
  masks the canonical defect.

## Change accounting

- new production branches/configuration: zero
- deleted obsolete contracts: exact-zero virtual-progress post-check and the
  unrepresentable exact-zero velocity assumption at the physical trajectory
  boundary
- remaining legacy authority: unchanged
- rollback commit: `992be2a`

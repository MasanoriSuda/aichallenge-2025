# Validation

## Dynamic evidence

Run `output/20260824-154559/d1/autoware.log` was bounded and stopped after the
first independent active-ShiftOut solver failure.

- The former `lateral tracking tube unavailable at state 18` discontinuity was
  removed.
- Nine fresh canonical Overtake plans completed the full selection chain and
  were stored.
- Production published a retained certified five-state ShiftOut command at
  about `4.35 m/s`, `+1.37 m/s2`, `+0.19 rad`.
- A later full-horizon solve reached OSQP maximum iterations at the first
  curvature-rate row.  This is recorded as a separate unresolved failure and
  was not hidden by tuning or fallback.

The follow-up run for the pre-entry consumer repair could not start AWSIM.  In
both `output/20260824-160211` and `output/20260824-160555`, the Unity process
remained at its memory-configuration banner, `/admin/awsim/start` had no
subscriber, and the host reported that `nvidia-smi` could not communicate with
the NVIDIA driver.  These runs provide no controller acceptance evidence and
are not counted as MPCC failures.

## Static validation

- `make autoware-build`: passed, 25 packages.
- `test_mpcc_progress`: passed.
- `test_persistent_osqp`: passed.
- `test_single_authority_source_contract.py`: 17 passed with pytest plugin
  autoload disabled.
- The normal CTest wrapper for the Python source-contract test is currently
  blocked by an existing unrelated collection failure:
  `test_localization_scope.py` cannot import `localization_scope`.  The two C++
  CTest targets pass in the same ROS-sourced container.
- `git diff --check`: passed.

## Reboot follow-up observation

Run `output/20260824-165722/d1/autoware.log` completed the requested observation
after the host reboot.  The host and simulator container both exposed the RTX
3090, AWSIM reached `grounded`, `ready` and `start`, and the bounded run was
stopped after the first active-ShiftOut failure chain.

- `Idle -> ShiftOut` occurred at decision 5997 / waypoint 172.
- Twenty fresh canonical candidates passed the complete chain and were stored
  during the following second.
- The first recorded failed solve used a bounded `18/20` horizon with state 19
  as the first unavailable tracking state.
- The first curvature box `[-0.353141, 0.353141]` and curvature-rate interval
  `[-0.142613, -0.110411]` have the non-empty intersection
  `[-0.142613, -0.110411]`.
- The maximum-iteration final iterate was primal-feasible under the physical
  row contract: the worst row was a heading dynamics equality with `1.05e-7`
  violation against `1.139e-3` tolerance.  OSQP still rejected the solve
  because its dual residual was `0.0233` at iteration 4000.
- No uncertified iterate executed.  Plan 6024 was retained until current-world
  proof rejected its initial corridor, after which Emergency and Recovery took
  authority.

This rejects an empty first curvature input intersection as the cause of this
failure.  It classifies the immediate solver rejection as an optimality/dual
convergence failure, not a demonstrated physical infeasibility.

## Next causal observation

Do not accept the maximum-iteration iterate and do not increase iterations.
First distinguish warm-start lineage from intrinsic QP conditioning:

- compare the same failed QP with a cold solve and an offset-aware warm solve;
- report the stage-geometry offset used to align the warm start;
- require offset zero when the first stage geometry is unchanged;
- require the exact observed offset when the horizon advances multiple stages.

The current compatibility check finds the rolling geometry offset but returns
only a boolean.  The solver then shifts every certified warm start by exactly
one stage.  During this run roughly 20 fresh solutions were stored while the
tracking waypoint advanced only from 172 to 176, so a one-stage-per-solve warm
advance is not justified by the physical horizon.

Do not change wall margin, steering-rate limit or OSQP iteration settings.  The
next slice must repair or falsify warm-start stage lineage before tuning.

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

## Required next observation

Once AWSIM can initialize, stop the first run after the first active Overtake
solve rejection and inspect:

- `horizon/configured` and `first_unavailable_state`;
- first `kappa_box`, `kappa_rate`, and `kappa_admissible` intersection;
- observed `x0_ey` and state-1/state-2 lateral tubes;
- warm-start/reset provenance;
- whether a bounded left/right pre-entry branch reaches canonical extraction.

Do not change wall margin, steering-rate limit or OSQP iteration settings until
that evidence distinguishes structural reachability from convergence failure.

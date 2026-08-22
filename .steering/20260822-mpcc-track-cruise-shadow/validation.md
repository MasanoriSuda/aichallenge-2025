# Slice 2 validation

## Result

The shadow implementation is accepted as diagnostic infrastructure, but Track/Cruise authority
promotion is blocked. The production command remained on `legacy-normal-bypass`; no shadow result
was selected or published.

The six-lap run proves that the five-state problem is generally solvable and fast enough to inspect,
but it also exposes two upstream contract defects which cannot be repaired by parameter tuning:

1. the circular reference path contains a duplicated endpoint, while the solver and physical
   certificate use different effective stage distances at that seam;
2. the converted five-state stage-1 predicted velocity is not semantically equivalent to the
   legacy target-speed command used for the command-difference metric or a future handoff.

Slice 3 must not promote authority until both contracts are defined and tested.

## Static evidence

- Failure-first focused test initially failed because the shadow eligibility and warm-start APIs did
  not exist.
- `make autoware-build`: passed after final authority-boundary audit, 25 packages in 4 min 34 s,
  Release build.
- Focused `test_race_mpcc_foundation`: 11/11 passed.
- Full package `ctest`: 33/33 passed in 17.24 s after the final audit correction.
- `git diff --check`: passed.
- Production configuration changes: none.
- Production authority/output branches added: none; the shadow has a dedicated solver context and
  reports `authority=shadow, selected=0`.

## Dynamic evidence

- Run: `output/20260822-124134`
- Mode: fixed single-car `make dev`, stopped after six completed laps.
- Lap times: 41.945, 38.524, 38.389, 38.504, 38.779, 38.779 s.
- Eligible cycles: 10,777.
- Metadata/build/attempt/solve/finite/constraint/convert/physical-check coverage: 10,777/10,777.
- Physically certified: 10,147/10,777 = 94.15%.
- Warm starts/resets: 10,769/8.
- Shadow timing:
  - build average/max: 0.040/0.377 ms;
  - solve average/max: 1.191/12.471 ms;
  - physical certificate average/max: 2.091/4.655 ms;
  - total average/max: 3.341/15.689 ms;
  - worst one-second-window total p95/p99: 13.200/15.689 ms.
- Production callback: 11,390 cycles, weighted average 5.786 ms, max 27.618 ms, three 25 ms
  overruns. There was no consecutive-overrun sequence in the once-per-second reports, but zero
  attributable overhead has not been proven.
- Immediate outcome transitions observed:
  - certified: 18 transitions;
  - heading unavailable: 6 transitions;
  - hard wall contact: 10 transitions;
  - swept wall path collision: 1 transition.

The transition counts are throttled state changes, not rejected-cycle totals. The aggregate
certificate count is the coverage source of truth.

## Root-cause evidence

### Circular seam geometry mismatch

`env/final_ver3/traj_mincurv_org.csv` has 350 data rows and its final `(x_m, y_m)` exactly duplicates
the first point. `mpc_stage_geometry::build()` preserves the resulting zero-length transition and
zero increment in `cumulative_distance_m`. `mpcc_progress::resolve_stage_distances()` separately
normalizes that zero transition to a positive minimum for the five-state dynamics. The physical
certificate still receives the raw cumulative distance and rejects `delta_distance <= 1e-6` as
`solution heading unavailable`.

Therefore the solver and certificate are not proving the same horizon at the circular seam. Raising
a tolerance or suppressing the warning would hide the defect and is explicitly rejected.

### Actuation semantic mismatch

`mpcc_progress::convert_extended_solution_to_legacy()` stores five-state stage-1 predicted velocity
in the legacy first input slot. The production legacy command slot represents a desired/target speed.
At launch the logged absolute difference reached 10.987 m/s because a dynamically reachable next
velocity was compared with the 11.11 m/s target. The difference remains material later in the run.

This does not prove the five-state trajectory is slow. It proves that one numeric slot currently has
two meanings. A future authority handoff needs an explicit actuation proposal containing target
speed, predicted velocity and acceleration rather than using this conversion as a command contract.

### Real physical rejections

The hard-wall and swept-path rejections away from the seam are not explained by the duplicated
endpoint. They show that some otherwise solved shadow candidates are physically inadmissible. The
certificate is working as intended; these candidates must remain non-executable and should be
revisited only after the common geometry contract is repaired.

## Acceptance decision

- Metadata coverage threshold: pass (100%).
- Solve/finite threshold: pass (100%).
- Physical certificate threshold: fail (94.15% < 95%).
- Shadow isolation: pass (`selected=0`, no config or output-authority change).
- Timing: diagnostic run acceptable, but three isolated callback overruns remain evidence for an
  asynchronous or budgeted execution slice before production authority.

Decision: retain and commit the shadow instrumentation; block Slice 3 authority promotion. The next
slice is a contract repair for canonical circular stage geometry and explicit MPCC actuation
semantics, still without production command authority.

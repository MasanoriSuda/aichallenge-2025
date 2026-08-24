# Validation

## Static gate

The physical proof remains observation-only. It cannot publish a command,
populate a canonical plan store or change normal authority. A solved artifact
must pass these boundaries in order:

1. immutable six-state artifact validation;
2. current intent and exact stage-geometry identity;
3. exact physical trajectory validation;
4. current reference-path course-frame reconstruction;
5. the established swept-footprint wall proof from the measured pose.

The adapter deliberately carries the solve-time course-progress origin and
nominal state-stage distances. No current-cycle consumer is allowed to infer a
different horizon for the old solution.

## Commands and results

- `make autoware-build`: passed for all 25 packages before the final cleanup.
- Full package test run: 45 CTest targets, 1,843 tests, zero failures or skips.
- Incremental controller rebuild after the final cleanup: passed.
- Targeted CTest rerun for the rate-resolved shadow, physical adapter and
  single-authority source contract: 3/3 passed.
- `git diff --check`: passed.

The existing stale `joycon_contract_guard/package.xml` result-parser warning is
unchanged and unrelated to this Slice.

## Dynamic gate

The first committed-source run, `output/20260825-034556`, failed this Gate
before wall acceptance. Both vehicles produced valid and publishable artifacts,
but most physical conversions were rejected as `progress-regressed`. Some
artifacts were accepted, proving that the adapter and wall checker can execute;
the dominant reject is conditional rather than a missing input. No callback
overrun or authority promotion occurred.

The diagnostic rerun, `output/20260825-035449`, confirmed the hypothesis. The
rejected transitions were microscopic solver-certified residuals, for example:

- delta `-8.41972e-08 m`, virtual-progress speed `-3.54755e-07 m/s`, dynamics
  defect `4.51999e-10 m`;
- delta `-1.68324e-09 m`, virtual-progress speed `-1.86816e-08 m/s`, dynamics
  defect `-9.66062e-12 m`.

The root cause was a contract mismatch: the QP accepts physical-row residuals
inside its reported certificate, while the downstream exact trajectory
validator required bitwise monotonic progress. The correction does not clamp
the primal or change the feasible set. The immutable artifact now carries the
physical virtual-progress bounds, proves the raw input and progress dynamics
against the existing solver certificate, and derives a bounded per-artifact
progress-regression tolerance for the exact physical validator. Existing
five-state callers retain the strict zero-tolerance default.

After the correction, `make autoware-build` passed for all 25 packages and the
full package test run passed all 45 CTest targets. A first test invocation
before the full rebuild exposed stale test binaries after the structure-layout
change; rebuilding the whole package removed the ABI mismatch and all tests
passed.

A committed-source rerun used `output/20260825-041116`. Across both vehicles,
32 telemetry windows contained 2,417 current-semantic physical evaluations:

- adapter rejects: 0;
- course-frame rejects: 0;
- wall rejects: 0;
- physical accepts: 2,417;
- every window remained `authority=shadow, selected=0`.

This accepts the physical-contract part of the Gate. It also exposed an
independent scheduling defect: the synchronous observation-only proof reached
10.485 ms and two D2 callbacks exceeded the 25 ms budget (26.993 ms and
25.614 ms, both attributed to the MPC region with `observation_only=1`). The
proof must be moved to a latest-only worker before any retained admission or
authority work. Reducing proof strength, cadence tuning or suppressing the
overrun warning is not an acceptable repair.

The next Gate requires:

- solved artifacts remain valid and publishable;
- current-semantic artifacts produce typed physical outcomes;
- no authority or command is selected from the six-state shadow;
- adapter/course-frame/wall rejection rates are classified rather than hidden;
- the same proof runs outside the control callback;
- completed proof results retain exact artifact/current-world provenance;
- the control callback has no observation-only overrun tail.

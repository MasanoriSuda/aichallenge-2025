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

The leading hypothesis is a solver-certified, tolerance-scale negative
virtual-progress/equality residual crossing an exact zero-tolerance monotonicity
boundary. The current log does not preserve the rejected progress delta or its
corresponding virtual-progress input, so no bound change or normalization is
authorized yet. Typed transition provenance is required before the rerun.

A rerun is pending. Acceptance requires:

- solved artifacts remain valid and publishable;
- current-semantic artifacts produce typed physical outcomes;
- no authority or command is selected from the six-state shadow;
- adapter/course-frame/wall rejection rates are classified rather than hidden;
- the synchronous current-world proof does not create a control callback
  overrun tail.

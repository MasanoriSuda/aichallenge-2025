# Validation

## Baseline

`output/20260822-161428`

- solved: 7,662 / 7,662
- physically certified: 7,627 (99.5432%)
- hard-contact reject: 26
- swept-path reject: 9
- example misclassification:
  `swept_index=0, swept_checked=1, stage=19, wp=281`

`swept_index=0` is the actual current pose, while `stage=19/wp=281` is stale diagnostic state.

## Required checks

- Unit: current index 0 resolves to current-pose origin and no horizon stage.
- Unit: index `k+1` resolves to horizon stage `k`.
- Unit: out-of-range index fails closed as invalid provenance.
- Runtime: no `current-pose-*` reject reports a non-negative horizon stage.
- Runtime: no swept path index 0 remains classified as candidate swept failure.
- Runtime: authority stays shadow and selected count stays zero.

## Results

### Failure-first

The first build failed in `test_mpcc_execution_contract.cpp` because the two current-pose reasons,
the failure-origin type and `resolve_swept_path_failure_origin()` did not exist. The implementation
was added only after that expected failure.

### Static validation

- `make autoware-build`: 25 packages passed.
- focused `test_mpcc_execution_contract`: passed.
- full `colcon test --packages-select multi_purpose_mpc_ros`: 33/33 targets passed.
- `colcon test-result`: 1,526 tests, 0 errors, 0 failures, 0 skipped.

### Primary dynamic run

`output/20260822-164756`, single-car `make dev`, approximately five laps:

- eligible / attempt / solved: 9,630 / 9,630 / 9,630;
- full physical certificate: 9,500 (98.6501%);
- candidate hard contact: 54;
- unsafe current production pose contact: 73;
- genuine swept candidate-connection failure: 3;
- invalid/bound/heading/sample/current-sample: 0;
- candidate certificate while current pose was safe: 9,500 / 9,557 (99.4036%);
- maximum solve / shadow time: 11.724 / 14.186 ms;
- callback maximum / overrun: 29.111 ms / 1;
- shadow selected: 0.

The old impossible combination `swept_index=0, stage=19` disappeared. Candidate discrete contacts
report their own stage without a swept index. A later production contact reports
`reason=current-pose-hard-wall-contact, stage=-1, wp=-1, swept_index=0`.

### Auxiliary runs

- `output/20260822-164021` exposed and then motivated removal of an accepted-current-pose diagnostic
  index leaking into later stage contact output. This was fixed before the primary run.
- `output/20260822-165950` did not progress beyond AWSIM `spawned` and was discarded.
- `output/20260822-170252` started normally but legacy production hit a wall near waypoint 51 and
  entered repeated stuck recovery. Shadow remained non-authoritative; this run is excluded from
  certificate-rate comparison and retained as a separate recovery/legacy-control artifact.

### Conclusion

The former swept-path bucket mixed two owners. The remaining Slice 2 work is now explicit:

1. remove candidate-created hard-contact and genuine swept failures;
2. define the safe handoff behavior when legacy production has already created an unsafe current
   pose;
3. do not relax the physical certificate or promote Track/Cruise authority before both are closed.

### Interface / authority audit

- Changed runtime files are limited to the participant package's internal MPCC certificate contract
  and controller diagnostics.
- No topic, service, message type, launch entry, result schema, ROS domain or submission layout
  changed.
- No solver formulation, wall margin, trajectory reference, command conversion, final publisher or
  fallback authority changed.
- Track/Cruise remains `authority=shadow, selected=0`; the dynamic runs contain no shadow selection.
- The certificate still fails closed on every condition that failed before this change. Current-pose
  failure is merely evaluated earlier and attributed to its actual owner.
- `aichallenge/result-summary.json` is user-owned and intentionally excluded from this change.

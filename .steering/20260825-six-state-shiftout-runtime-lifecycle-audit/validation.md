# Validation

## Static verification

- `make autoware-build`: 25 packages completed successfully.
- Full `multi_purpose_mpc_ros` package tests: 50 CTest targets and 1,881
  individual tests, with zero failures, errors or skips.
- `git diff --check`: passed.
- The existing stale `joycon_contract_guard/package.xml` result-parser warning
  is unrelated and unchanged.
- A host-side pytest collection was not used as evidence because the host
  environment lacks the unrelated `localization_scope` module; Docker is the
  repository's canonical ROS/test environment.

## Dynamic verification

Two initial `make dev2` runs (`output/20260825-233219` and
`output/20260825-233355`) did not enter ShiftOut and are excluded from this
Gate.  The bounded attributable run is `output/20260825-233538`.

The first accepted episode joined as follows:

- `1787668576.706099886`: `Idle -> ShiftOut`, target `d2`, generation 1,
  side -1;
- the same entry passed six-state Gate A and atomic admission;
- `1787668576.836761181`: the exact six-state execution source was promoted,
  age 0.015 s, 20 points, with swept-current-to-horizon wall validation;
- later episodes also promoted refreshed sources, including count 10 within
  one Mission generation.

This falsifies the original missing-source behavior, where every handoff
reported `latest=no solved trajectory; last_feasible=no last feasible
trajectory` despite valid six-state publications.

## Independent residual defect

After the first episode, fresh six-state solves eventually stopped.  The
dominant diagnostic was `failed_iterate_row=254`.  With six states, three
inputs and `N=20`, this row is the stage-zero virtual-progress-speed input box.

The same row and failure cascade already appear after the first ShiftOut in
the pre-change `output/20260825-231050`.  Therefore it is not a regression
caused by the new source projection.  The strict 0.5 s source freshness
contract correctly expired once fresh certified artifacts ceased.  Solver
formulation feasibility is the next root-cause Slice and is not masked here by
age renewal, fallback or parameter tuning.

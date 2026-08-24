# Validation

## Static and package gates

- `make autoware-build`
  - PASS: 25 packages.
  - The only stderr was the existing setuptools `setup.py install`
    deprecation warning.
- Docker `colcon test --packages-select multi_purpose_mpc_ros`
  - PASS: 40 / 40 CTest entries, 0 failures, 20.31 s.
  - `test_mpcc_progress`: 76 / 76 assertions.
  - `test_single_authority_source_contract`: 18 / 18 assertions.

The source contract fixes the first-transition anchor, immutable stage timing,
fresh/Emergency Rejoin selection and absence of a Rejoin legacy fallthrough.

## Dynamic evidence before root repair

`output/20260824-181825/d1/autoware.log`

- Rejoin solve: 38 / 38.
- Fresh physical/canonical selection: 0 / 38.
- Rejects: 10 future hard-wall, then 28 current-pose hard-wall.
- Production source: legacy three-state normal.

This run rejected authority promotion and motivated root-cause analysis.

## Rejected counter-hypothesis

`output/20260824-183322/d1/autoware.log`

A synchronous progress-indexed physical wall profile did not repair the
exact-vs-affine first-stage mismatch and raised normal callback averages to
roughly 29--37 ms with sustained overruns. The candidate was removed; it is
not retained behind a flag.

## Dynamic qualification after root repair

`output/20260824-191213/d1/autoware.log`

- First Rejoin cycle: one maximum-iteration solve miss.
- Following window: 33 / 33 complete fresh physical/canonical selections.
- Recovery exit: 1.

This qualified fresh Rejoin. It did not qualify retained Rejoin, so the
production policy remains fresh or explicit Emergency only.

## Post-promotion dynamic Gate

`output/20260824-192226/d1/autoware.log`

- Rejoin eligible/build/solve: 83 / 83 / 83.
- Fresh physical/canonical selections: 69 / 83.
- Exact physical rejects: 14 (13 hard-wall, 1 swept-path).
- Sampled final decisions: 12 canonical fresh, 9 explicit Emergency.
- Rejoin legacy-normal decisions: 0.
- Canonical contract join failures: 0.
- Recovery exit: 1 (`Recovery -> FollowPrepare -> Idle`).

The single-authority Gate passes: no Rejoin cycle can fall through to the
legacy normal solver. The physical-reject/Emergency tail and callback overruns
remain runtime-quality evidence for a later Slice; neither is hidden with a
retained plan, margin change, timeout or fallback.

The run was recorded immediately before correcting the aggregate telemetry
label, so its aggregate lines still say `shadow`. Final decision traces already
prove actual production ownership. The telemetry-only correction is covered by
the final build and source-contract test and now emits `authority=production`,
`production_authority=canonical` and the actual selected bit.

## Excluded runs

- `output/20260824-185824`: AWSIM remained `Spawned`; no valid race start.
- `output/20260824-190347`: manual initial-pose/control intervention changed
  the scenario; no valid Rejoin qualification.

These runs are not counted in any acceptance metric.

## Residual scope

- Pass and Return are not dynamically exercised by this Rejoin Slice.
- Overtake wall feasibility and callback timing are not declared production
  quality complete.
- No parameter, solver setting, margin, timeout, lease or fallback changed.

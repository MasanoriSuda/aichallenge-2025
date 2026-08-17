# Results

## Static verification

- `docker compose run -T --rm --no-deps autoware-build`
  - 25 packages built successfully.
  - The only stderr was the existing setuptools deprecation warning.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - 28 test targets passed.
  - Final package-scoped result: 1233 tests, 0 errors, 0 failures, 0 skipped.

## Implemented behavior

- Complete Frenet lateral profiles are validated with the same current-to-first
  and backward-difference heading convention used by execution.
- Execution repair contracts the complete profile toward two bounded fallback
  shapes instead of moving one waypoint independently.
- All bounded speed and wall-clearance candidates are exhausted before a hard
  physical failure is reported.
- A still-valid last-feasible or baseline profile may bridge a future-profile
  miss for one control cycle. Current wall contact, unknown/out-of-map,
  EmergencyBrake and solver guards remain hard.
- Existing periodic debug output now includes `profile_keep`.

## Dynamic acceptance criteria

Run `make dev2` without changing the opponent setup and compare with
`output/20260818-064806` over an equal duration:

- physical-revalidation failure events: lower than 14;
- static wall margin failure transitions: lower than 5;
- `Pass -> Return` occurs without a new wall-contact increase;
- `profile_keep < 1.0` appears only during bounded repair and does not chatter;
- no MPC solver-failure increase and no new unexpected speed cap.

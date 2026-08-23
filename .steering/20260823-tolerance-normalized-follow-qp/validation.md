# Tolerance-normalized Follow QP validation

## Static validation

- `make autoware-build`: succeeded after the final `S_i=T/t_i` transformation.
- `test_persistent_osqp`: 10/10 passed.
- Full `multi_purpose_mpc_ros` package test: 38/38 CTest entries passed.
- Aggregate result: 1620 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.

## Dynamic validation

Final run: `output/20260823-144839`, `make dev3`.

- Follow shadow attempts: 1322.
- Accepted: 1305 (98.7%).
- Attempt-weighted solve time: 2.56 ms average, 30.87 ms maximum.
- Legacy execution-primal rejection: 0.
- Stopped-target windows: 3/3, 41/41, 41/41 and 6/6 accepted.
- Authority remained `shadow`, `selected=0`; no production promotion occurred.

For comparison, baseline `output/20260823-140735` accepted 526/603 (87.2%),
reported 35 execution-primal rejects and took 4.41/35.95 ms average/maximum.
Traffic and production Overtake state differ between runs, so this comparison is
limited to the Follow shadow telemetry contract.

## Gate decision

Keep the preconditioning policy connected to Follow shadow for further evidence.
Do not promote Follow production authority: 17 attempts remained unaccepted and
the 30.87 ms worst case exceeds the 25 ms control period.

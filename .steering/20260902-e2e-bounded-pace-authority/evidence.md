# Evidence

## Infrastructure audit

The first requested seed-2035 run was rejected before evaluation.  Although
the Make target exported `E2E_START_RANDOM_SEED=2035`, Compose placed that
variable only on the physical racing-kart-interface base, not on the simulator
service.  Process provenance showed AWSIM actually running
`--start-random-seed 2026`.  The variable now belongs to the shared Autoware /
simulator base beside `SIM_MODE`; it was removed from the unrelated vehicle
interface base.

This also means earlier runs labelled seed 2034/2035 exercised repeated default
seed 2026 trials.  They remain paired acceleration evidence, but are not valid
cross-seed evidence.  No production promotion may rely on those labels.

## Closed-loop Gates

The first bounded run was rejected as seed evidence after process inspection
showed seed 2026.  Both admitted runs below were verified from the live AWSIM
command line and, for seed 2036, the normalized effective-seed startup log.

| Run | Actual seed | Finish | Penalty | Stall | Total | Mean / max speed |
|---|---:|---:|---:|---:|---:|---:|
| `output/20260902-e2e-bounded-pace-seed2035-rerun` | 2035 | 3/3, P1 | 0 | 0.0 s | 256.488 s | 3.790 / 4.456 m/s |
| `output/20260902-e2e-bounded-pace-seed2036` | 2036 | 3/3, P1 | 0 | 0.0 s | 255.873 s | 3.805 / 4.451 m/s |
| `output/20260902-e2e-bounded-pace-packaged-seed2037` | 2037 | 3/3, P1 | 0 | 0.0 s | 255.648 s | 3.800 / 4.459 m/s |

Both `analyze_e2e_run.py --fail-on-stall` and
`analyze_e2e_competition.py --fail-on-rejection` passed.  Runtime provenance
matched `fixed_lidar_brake`, acceleration `0.8 m/s2`, speed limit `4.6 m/s`
and the packaged TinyLidarNet checkpoint.  The first two override totals differ
by only 0.615 s.  A third run with no acceleration or speed-limit environment
override proved that the promoted package defaults produce the same behavior.
All three totals span only 0.840 s.  The old `0.6 m/s2` default-seed reference
was 292.483 s; it remains valid as paired pace evidence but not as cross-seed
evidence.

## Verification

- host participant/system launch contracts: 16 passed
- focused controller tests in Docker: 48 passed
- participant/system launch contracts in Docker: 3 + 13 passed
- focused competition analyzer tests in Docker: 13 passed
- Docker ML suite: 210 passed
- selected-package `colcon test`: controller 40 + 8, participant launch 2 + 3,
  system launch 13 passed
- `make autoware-build`: 25 packages completed
- shell, Python, XML, Compose and `git diff --check`: passed

## Decision

Promote the bounded pair, not unbounded acceleration: production launch and
parameter defaults become acceleration `0.8 m/s2` plus maximum forward speed
`4.6 m/s`.  The governor remains independently configurable and fail-closed;
the lateral model, steering authority and obstacle distances are unchanged.

# Evidence

## Frozen-bag replay

Source: `output/20260901-090729/d4/rosbag2_autoware`.

The old teacher remained `front-clear` from 105 through 129 s because its
whole-sector 10th percentile stayed between 2.1 and 3.4 m.  The run-level
positive-acceleration stall began at 129.65 s.  The nearest supported right
side return was already about 1.5--1.6 m during several of those samples.

The pre-contact teacher replay changed representative commands as follows:

| time | right cluster | ML base steer | projected steer | decision |
|---:|---:|---:|---:|---|
| 105 s | 1.55 m | -0.243 rad | +0.176 rad | side-clearance |
| 113 s | 1.49 m | -0.351 rad | +0.218 rad | side-clearance |
| 119 s | 1.50 m | -0.336 rad | +0.215 rad | side-clearance |
| 129 s | 1.63 m | -0.256 rad | +0.122 rad | side-clearance |

Positive steering is away from the right-side obstacle.  The candidate thus
reacts more than 24 s before the frozen stall boundary without changing the
1.8/0.9 m physical thresholds.

The source scan was also decoded directly: 750 samples, angle_min
-1.566607475 rad, angle_max +1.570796371 rad, 179.76 degree FOV.  This rejected
an initial hypothesis that the teacher's 180 degree angle model mismatched the
runtime scan.

## Closed-loop final-world audit

- command: `make e2e-final-precontact-teacher`, then
  `make awsim-request-start` after all four domains reported `Grounded`
- run: `output/20260901-092811`
- d1--d3: production `fixed_lidar_brake`
- d4: diagnostic `precontact_teacher`
- all modes were verified in launch logs before start
- run stopped manually after more than 600 s because the four-vehicle AWSIM
  world did not publish a terminal state despite its configured timeout

| domain | lateral owner | distance | duration | longest low speed | positive-accel stall | result |
|---|---|---:|---:|---:|---:|---|
| d1 | production student | 56.61 m | 606.61 s | 534.34 s | 259.99 s | fail |
| d2 | production student | 2026.34 m | 606.90 s | 0.00 s | 0.00 s | pass |
| d3 | production student | 1839.38 m | 606.63 s | 29.56 s | 1.59 s | fail |
| d4 | pre-contact teacher | 1949.30 m | 606.70 s | 0.00 s | 0.00 s | pass |

At 157 s, while the historical d4 was physically stuck, the diagnostic d4 was
moving at 3.72 m/s.  At 231 s it was moving at 4.28 m/s.  It remained free of
both low-speed and positive-acceleration stall intervals for the whole
606.70 s bag.  Teacher intervention returned to zero in clear intervals, so it
did not retain lateral authority continuously.

## Root cause

The failure was not produced by the side distance threshold itself.  It was
produced by two upstream operators:

1. whole-sector percentile aggregation hid a narrow but coherent peer return;
2. residual blending allowed a large ML command to remain directed toward the
   peer after side risk was detected.

Nth-nearest clustered sensing plus directional projection addresses those two
operators one-for-one.  The closed-loop removal of d4's 84 s stall supports the
causal explanation.

## Admission decision

The pre-contact teacher is accepted as a **diagnostic candidate**, not yet as a
training source or production controller.  This run lacks AWSIM Finish/contact
evidence and the unchanged production d1 entered a separate contact trap.
Before label extraction, all-teacher or otherwise result-producing validation
must prove Finish, contact acceptance and run-level stall gates.  Production
checkpoint and runtime defaults remain unchanged.

## Verification

- focused Python tests: 34 passed
- `make autoware-build`: 25 packages built successfully
- `make -n e2e-final-precontact-teacher`: correct d1--d3/d4 mode split
- frozen-bag offline replay: pre-stall activation and safe steering direction
- closed-loop 4-domain bags: d4 stall gate passed
- `git diff --check`: clean

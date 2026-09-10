# Nine-state dev2 first failure and bounded comparisons

Production HEAD `b0478348fda0986ae46c005fd574852154f4d6a1`; no production
solver/model/limit changes during these comparisons. Full M3–M6 acceptance is open.

## Actual runtime

`output/20260910-nine-state-single-r2`: six laps241.24130249023438s, penalty0,
no active moving Emergency/Recovery and no missing causal observations.
Public control receipt40.0071Hz, maximum34.1609ms;11755callback records have
weighted mean4.99412ms, maximum26.879ms and one overrun. Per-cycle p95/p99 were
not recorded.12duplicate command source intervals and maximum115ms source interval
still need source-clock attribution; receipt and source intervals are distinct.
242moving control-origin prediction anchors: position MAE0.00954866m, maximum
0.0331256m. This nominal0.13s live-prefix score is not a1s open-loop bound.

`output/20260910-nine-state-dev2-r1`: first D1moving Emergency938, source
10.094999774s, control10.224999774s, observed speed about1.65m/s. Current world
fingerprint5538338864624173385 (`4cdc21da9faa0149`), inspected artifact378.
The run was rejected and torn down; no six-lap acceptance is claimed.

Exact zero-solve replay in `output/20260910-nine-state-dev2-revalidation-r1`:
937retained terminal peer clearance+0.0185712315m;938−0.00170420267m.
The independent immediate Stop also rejects,−0.000366633626m. Continuation,
wall and steering join pass. Substituting only the previous peer observation,
projected to938, makes the retained terminal pass+0.00501269m; substituting the
new peer at937 rejects. Preserving only the previous publication anchor does
not repair938. No margin or tolerance was changed.

## Solver/formulation comparisons

- Default A/B/C/D and live normal left/right reject on the same world.
- KKT-only and whole-pipeline KKT variants reject. A solved affine QP can still
  yield a wall-contacting nonlinear path. `dynamic-kkt-pipeline-r1` also rejects
  the next coupled QP at4000iterations.
- `independent-qp-r1`: original physical dynamic and coupled QPs are feasible.
  HiGHS maximum violations2.43e-10/7.31e-9; ProxQP2.31e-6/2.18e-6. These are
  affine results only. No backend was promoted.
- HiGHS full pipeline r3: five optimal QPs, dual stationarity about6.6e-7;
  full physical wall proof rejects at stage339/wp35. r4common-primal and
  r5physical-rollout tangent/peer/wall loops reject an infeasible next affine QP.
  r5initial nonlinear/primal state maximum difference0.0641809. Its row-group
  diagnostic implicates the dynamics/peer intersection, not physical impossibility.
  The deletion-filter subset was not independently certified irreducible.
- HiGHS r1 mistakenly passed the upper Hessian triangle that this installed
  implementation ignored; r2square format was unsupported. Both are invalid
  backend comparisons. r3transposes the original upper triangle, preserves all
  nonzero coefficients and independently checks stationarity. HiGHS1.7.2 is
  local diagnostic-only, not a production dependency.
- `production-normal-r1` calls the actual bounded Cruise avoidance producer.
  OSQP rejects both sides; HiGHS left reaches exact wall rejection, right rejects
  the obstacle QP. The earlier physical-dynamic-SQP tool's Overtake-only G arms
  are not applicable to Cruise and must not stand in for these live candidates.

## Stop comparison coverage correction

The historical `compare_stop_physical_support` and Stop schedule tools use
`build_current_world_maximum_braking_candidate`. The live worker instead uses
`build_current_world_complete_rest_candidate`, with freely optimized inputs and
20stages of0.25s here. Historical maximum-braking outcomes do not establish the
live complete-rest outcome. A canonical `--current-world-complete-rest-only` entry now exercises the live
producer, with a coupled rear/side-peer regression comparing live and audit
identities/proofs. Build r11 passes26packages; tests r8 pass2388records with
zero errors/failures/skips. The new CLI reproduces candidate334c7985b802506b
and the same expected dynamic QP rejection. See `current-stop-tool-checks.json`.
Do not modify production from the obsolete comparison alone.

`production-stop-r1` directly calls the live producer. Its original inherited
racing objective rejects the dynamic QP with OSQP and HiGHS. Objective-only
feasibility also rejects: OSQP dynamic QP; HiGHS reaches exact rest rejection after
all3existing corrections. The live producer currently retains the source weights;
earlier progress text that broadly called every Stop a zero-cost feasibility
problem was inaccurate. Zero cost applies to the historical max-braking producer.

`stop-target-audit-r1` changes only the hypothetical terminal feedback target
in the zero-solve retained938replay.43targets−3..1.2m were tested. Targets
0.8/0.9/1.0/1.1/1.2m pass the existing continuation, complete-rest, wall and peer
checks, with clearance0.00750/0.01672/0.02348/0.02827/0.04259m. The original target
rejects. These are candidate terminal policies, not adopted normal authority.
Their certificate ends when rest is reached; it is not a five-second stationary
occupancy guarantee. No arbitrary constant target is proposed for production.

Coherent hold-steering/max-braking seeds and three fixed-rate schedules still
reject the historical constrained Stop. `stop-shooting-r1/r2` instead use the
live complete-rest5s grid and all current hard input/world constraints.225constant
and576single-switch lateral-feedback/power-then-brake input sequences give no
physical witness: respectively157/37/31and210/285/81dynamic/wall/artifact-bound
rejections. No QP was solved by these independent physical oracles. This bounded
population failure is Unknown, not an infeasibility certificate.

## Public peer observation and next causal work

`peer-observation-r1` uses only the same run's public V2X messages for scoring.
The world peer starts about10mm from the interpolated public position. After
matching that initial offset, constant-Cartesian-velocity position error is
0.01446m@0.1s,0.04871m@0.25s,0.13849m@0.5s,0.44233m@1s,1.61082m@2s,
8.10457m@5s. Future positions are diagnostic observations only; after Emergency,
the other vehicle may respond to actual behavior. They are not control inputs or
a guaranteed counterfactual future. Investigate causal public-history/known-map
prediction before changing the peer producer, with a frozen holdout comparison.

The clock-regression path already clears velocity/yaw/tire observation deques
along with command history (`mpc_controller_cpp.cpp` control epoch handling).
The remaining task is a causal reset/restart exercise, not an assumed missing clear.

All native/public/solver evidence is empirical2025-derived simulation evidence.
User JSON, original simulator DLL and existing artifacts remain protected.

## Terminal-time candidate repair

The newer [terminal-time design](stop-terminal-time-design.md) records a positive
0.8s complete-rest solve under the unchanged physical scene. The live/audit
population now owns native braking and maximum-support clocks with zero-cost
feasibility. Per-clock artifact identity, publication-period floor, all peers,
supersession and maximum-support rear-peer proof are covered. Buildr13passes26
packages; package r9passes2389records/0errors/failures/skips. Canonical938replay
accepts the first candidate. Fresh dev2acceptance is pending; no completion claim.

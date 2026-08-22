# Current authority graph

## Status vocabulary

- **Confirmed**: directly supported by the baseline source, history, or runtime log.
- **Observed conflict**: the runtime trace explicitly reports competing owners or formulation changes.
- **Partial**: a guard exists, but it does not cover the full observation-to-command chain.
- **Unknown**: the current evidence cannot prove the contract.

## Observation-to-command flow

```text
/localization/kinematic_state ----+
/planning/.../trajectory ---------+
/v2x/vehicle_positions -----------+--> MpcControllerCpp callback
map/path constraints -------------+          |
/awsim/state,status --------------+          v
                                      race/session state
                                               |
                                               v
                                      V2X behavior / target
                                      Follow/Cruise/Overtake
                                               |
                  +----------------------------+--------------------------+
                  |                            |                          |
                  v                            v                          v
        OvertakeLine Mission          Dynamic Escape attempt      low-speed avoidance
        phase/target/side             target/side/lease            direct target/control
                  |                            |
                  +--------------+-------------+
                                 v
                      async tactical worker
                      left/right Mission + score
                                 |
                                 v
                      Frenet DP stage corridor
                                 |
                                 v
                         init_problem()
                reference + bounds + speed owners
                                 |
              +------------------+------------------+
              |                                     |
              v                                     v
      5-state extended MPCC                3-state/legacy problem
      [ey,elag,epsi,v,theta]               [ey,epsi,s-or-t]
              |                                     ^
              | convert_extended_solution_to_legacy|
              +------------------+------------------+
                                 v
                   nonlinear/physical wall checks
                                 |
                +----------------+----------------+
                |                                 |
                v                                 v
          solved prediction                 wall/solver hold
                |                                 |
                +-----------+---------------------+
                            v
                 command post-processing
             acceleration/steering filters and caps
                            |
                            v
                stuck recovery arbitration
                            |
                            v
                final control source resolver
                            |
                            v
              /control/command/control_cmd
```

## Current owners and bypasses

| Stage | Current owner/producer | Can change authority | Bypass or alternate path | Evidence |
|---|---|---|---|---|
| Ego observation | odometry callback | stale/non-finite failsafe | direct failsafe publish | `mpc_controller_cpp.cpp:43692-43709`, `48923-48928` |
| Reference geometry | `ReferencePath`, current waypoint association | path reset/reassociation | global association after local loss | `docs/spec/mpc-integration.md` correctness-hardening section |
| Target/behavior | V2X behavior and OvertakeLine state | target continuity, phase and Recovery transitions | Dynamic Escape may run while phase is Idle | `mpc_controller_cpp.cpp:17932-17938`; runtime decision 1386 |
| Tactical branch | latest-only async worker | left/right selection, cached result admission | existing Mission or racing line remains live | `mpc_controller_cpp.cpp:7332-7398` |
| Lateral corridor | Mission profile / Frenet DP / racing line / recovery line | rolling refresh, wall replan, DynamicWait | same-side retained prefix | config lines `732-769`; authority resolver |
| Solver formulation | `init_problem()` and extended solve path | overtake phase, Dynamic Escape lease, preparation, circuit breaker, reentry | extended -> 3-state -> legacy | `mpc_controller_cpp.cpp:17932-17960`, `20822-20945` |
| Physical certificate | entry and executed-solution wall validators | may hold, rollback, or request replan | last validated/retained solution | `mpc_controller_cpp.cpp:20948-20960`, commits `9618f40`, `0561f57` |
| Low-speed pass | direct low-speed controller | bypasses the normal QP | direct velocity/steering, then solver handoff | `mpc_controller_cpp.cpp:20813-20817`, `20576` |
| Solver failure | safe failure, continuation, crawl | may retain steering, crawl, or stop | no single canonical same-formulation rule | final-source resolver and `mpc_controller_cpp.cpp:48955-49005` |
| Command post-process | controller callback | acceleration policy, filters, steering gain/rate limits | can alter solved first input | `mpc_controller_cpp.cpp:48815-48921` |
| Recovery | stuck recovery supervisor | can discard normal control and own gear/command | intentionally separate | `mpc_controller_cpp.cpp:48867-48914` |
| Final publish | final source resolver + publisher | twelve enumerated sources | one publisher, multiple logical owners | `overtake_execution_orchestrator.cpp:666-723` |

## Formulation transition graph

```text
Normal Track/Cruise/Follow
    -> legacy MPC

ShiftOut/Pass/Return or validated Dynamic Escape
    -> progress preparation
       -> extended 5x3 solve
          -> converted to legacy output layout
       -> if unavailable/requalifying: 3-state progress solve
    -> if progress preparation rejected: legacy elapsed-time MPC

Any solve/physical validation failure
    -> bounded continuation / wall hold / crawl / forced stop
    -> possibly Recovery
```

Confirmed source facts:

- `progress_contouring_mpcc_overtake_only: true` at `config/config.yaml:332`.
- preparation reject explicitly logs `using legacy MPC for this cycle` at
  `mpc_controller_cpp.cpp:17954-17960`.
- extended failure explicitly logs `using 3-state MPCC` at
  `mpc_controller_cpp.cpp:20914-20918`.
- successful extended output is converted by `convert_extended_solution_to_legacy()` at
  `mpc_controller_cpp.cpp:20858-20867`.

## Runtime evidence: `20260822-105057` D1

| Log line | Observation | Classification |
|---|---|---|
| 160, 203, 233, 317 | normal/Follow cycles use `solver="legacy-mpc-solved"` | Confirmed overtake-only scope |
| 377 | Dynamic Escape uses `extended-mpcc-solved` | Confirmed extended execution |
| 387-389 | fresh execution gap/identity mismatch enters wall hold and publishes `-3.00 m/s2` | Mask/authority handoff evidence |
| 394-396 | next cycle accepts a fresh Dynamic Escape solution and resumes acceleration | Observed command-source churn |
| 1511-1519 | retained Dynamic Escape stage bridges into ShiftOut through wall requalification | Partial continuity contract |
| 1580 | DynamicWait uses `legacy-mpc-solved` and publishes `-3.00 m/s2` | Cross-formulation transition |
| 1590 | trace reports `conflict=multiple-lateral-authorities` | Observed authority conflict |

The log proves the mixed-authority structure is active. It does not yet prove which individual
transition caused the visible driving failure; that requires deterministic replay in the relevant
migration slice.

## Current safety certificates

Existing checks worth preserving include:

- solver status, finite solution, and maximum constraint violation;
- target provenance and asynchronous context checks;
- stage corridor and physical wall/footprint validation;
- execution trajectory wall validation;
- target/side/attempt identity for retained Dynamic Escape execution;
- odometry/non-finite failsafe;
- Recovery swept-footprint and V2X clearance checks.

The gap is not absence of checks. It is that certificates are attached to several representations
(Mission, DP prefix, converted solution, current prediction, retained execution) instead of one
immutable `CertifiedMpccSolution` consumed by the publisher.

## Target authority graph

Normal operation should have one path:

```text
snapshot -> intent -> canonical MPCC problem -> solve -> certify -> publish
```

Only these overrides remain:

```text
emergency safety fault -> stop command
confirmed stuck/contact/gear episode -> recovery command
```

The tactical worker, DP corridor, wall evaluator, and Mission FSM remain useful components, but none
may separately own a normal lateral or longitudinal command.

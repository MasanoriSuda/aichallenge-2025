# Phase 0 design

## Executive summary

The current implementation already contains the core ingredients of a race MPCC: a five-state
velocity-progress formulation, dual left/right evaluation, stage-wise wall/opponent corridors,
physical trajectory validation, provenance checks, retained feasible execution, and decision traces.
It is not yet a single-authority controller.

The primary architectural gap is that the five-state MPCC is an overtake-scoped optional formulation
inside a larger legacy controller. When it is not requested, cannot be prepared, is requalifying, or
fails, execution can move through a three-state progress formulation or legacy MPC. Low-speed direct
control, wall holds, solver continuation/crawl, final acceleration/steering post-processing, and stuck
recovery can also own or alter the command.

This makes a failure difficult to localize: candidate generation, solver formulation, physical
validation, and final command authority may refer to different representations or different cycles.
The target is therefore not "turn MPCC on everywhere". The target is to reduce normal-driving
authorities to one certified MPCC solution and preserve only explicit emergency and recovery
supervisors outside it.

## Confirmed current facts

1. `config/config.yaml:331-332` enables contouring MPCC but restricts it to overtake.
2. `src/mpc_controller_cpp.cpp:17932-17960` falls back to legacy MPC when progress preparation is
   rejected.
3. `src/mpc_controller_cpp.cpp:20855-20945` converts a five-state extended solution to the legacy
   solution layout, and otherwise solves the older problem.
4. `src/overtake_execution_orchestrator.cpp:666-702` resolves twelve final control sources, including
   direct control, multiple hold/fallback forms, recovery, and failsafe.
5. `src/mpc_controller_cpp.cpp` is 49,285 lines at the baseline, with planning, solving, validation,
   lifecycle state, arbitration, recovery integration, and publishing sharing one translation unit.
6. The current `config.yaml` contains 54 `v2x_overtake_mpcc_*` settings. Count alone is not a defect,
   but it shows that migration and tactical policy are extensively runtime-configurable.

## Runtime observations at the baseline

`output/20260822-105057/d1/autoware.log` shows:

- decision 1386: Dynamic Escape publishes `extended-mpcc-solved`;
- decision 1392: the next handoff publishes a wall-admission hold and `-3.00 m/s^2`;
- decision 1393: execution returns to `extended-mpcc-solved`;
- decision 3504: `DynamicWait` publishes `legacy-mpc-solved`;
- decision 3514: the trace reports `multiple-lateral-authorities` while the solver is again
  `extended-mpcc-solved`.

This evidence does not prove that every transition is unsafe. It proves that solver formulation,
tactical state, lateral owner, longitudinal owner, and final source remain independently variable in
one encounter. That is the integration surface Phase 1 must make explicit.

## Target architecture

```text
ObservationSnapshot
  ego / reference / V2X / map / race session
                     |
                     v
MissionSupervisor -> ControlIntent
  target / homotopy / no-return / completion semantics
                     |
                     v
DynamicEnvironment
  stage geometry / wall bounds / obstacle tubes / velocity bounds
                     |
                     v
CanonicalMpccProblem(problem_fingerprint)
                     |
                     v
CanonicalMpccSolver + same-formulation warm start
                     |
                     v
CertifiedMpccSolution
  solved / finite / constraint residual / physical certificate / validity horizon
                     |
          +----------+-----------+
          |                      |
          v                      v
EmergencySupervisor      RecoverySupervisor
  stop-only override      stopped/contact/gear/reverse only
          |                      |
          +----------+-----------+
                     v
FinalControlDecision -> /control/command/control_cmd
```

The Mission supervisor chooses intent and topology; it does not emit steering or velocity. Corridor
generation remains a planning component but becomes an input to the canonical problem, not another
execution authority. Emergency and recovery are intentionally outside the forward-racing MPCC.

## Audit method

The audit follows the command path upstream from the published command and separately follows each
piece of state downstream from its producer. For each transition it records:

- owner;
- problem/context fingerprint;
- certificate;
- bypass path;
- failure transition;
- whether the transition changes formulation;
- downstream mechanism that masks the original failure.

History is inspected by patch family, not by assuming every existing branch is required. The patch
ledger records the symptom it addressed and the condition under which it can be removed.

## Architectural decisions for later phases

These are Phase 0 decisions, not production implementation:

1. The canonical target is the five-state velocity-progress MPCC, subject to Phase 1 verification.
2. The existing three-state and legacy formulations are migration authorities, not permanent
   fail-operational peers.
3. A last-feasible bridge may remain only when it is a certified solution from the same formulation
   and matching problem context.
4. Physical wall/opponent validation is retained, but its result becomes part of the solution
   certificate rather than a separate competing path identity.
5. Stuck recovery remains separate. Low-speed pass direct control does not.
6. Parameter tuning starts only after the relevant slice removes the competing authority and passes
   deterministic replay.

## Why a full rewrite is rejected

A complete rewrite would discard working safety checks, trajectory interfaces, V2X continuity,
recovery, and evaluation contracts at once. The migration therefore uses vertical slices. Each slice
adds a failing replay, repairs one invariant, promotes one intent family, and deletes the replaced
production branch before proceeding.

## Documentation placement

- This steering contains current-baseline findings and the migration proposal.
- Package `AGENTS.md` contains stable engineering rules.
- The local skill contains the repeatable audit workflow.
- `docs/spec/mpc-integration.md` remains the current implementation specification. The target
  architecture should be promoted there only after the user accepts the Phase 0 audit and a Phase 1
  slice establishes the new contract.

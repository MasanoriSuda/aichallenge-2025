# Slice 1 design

## Causal statement

The final publisher currently receives a numeric command and a collection of independently produced
status fields. Because the exact problem context and physical certificate do not travel with the
command, downstream logs infer identity from `decision_id`, target/side, source enums, and free-form
reason strings. Retained execution and post-solve arbitration can therefore change the actual command
source without changing all of those fields. The observed symptom is that a bad transition can be
seen, but the exact producer/certificate that authorized it cannot be proven.

Slice 1 repairs the missing identity carrier. It deliberately does not repair the mixed-authority
architecture yet.

## Contract model

### `ControlIntent`

Represents Track, Cruise, Follow, Hold, Stop, ShiftOut, Pass, Return, and Rejoin. It describes what
the supervisor requests; it does not own a command.

### `MpccProblemContext`

Contains:

- control decision and intent generation;
- observation and target/obstacle generation;
- stage-geometry fingerprint;
- target identity;
- horizon size;
- formulation and state/input schema;
- bounds and cost schema IDs.

The context fingerprint is deterministic and excludes mutable output state.

### `CertifiedMpccSolution`

Contains:

- monotonic solution ID;
- source context fingerprint;
- formulation;
- solve, finite, constraint and physical-certificate status;
- maximum constraint violation;
- prediction stage count and validity horizon.

`certified()` is true only when all required predicates hold.

### `FinalControlDecision`

Classifies the actual final authority as one of:

- certified normal solution;
- legacy normal bypass;
- Emergency override;
- Recovery override;
- disabled control.

`identity_complete` answers whether the actual producer is traceable. `canonical_contract_satisfied`
is stricter: it is true only for a matching certified normal solution or an explicit permitted
override. A legacy bypass can therefore be completely identified while correctly remaining a target
architecture violation.

## Runtime integration

1. `MPC::get_control()` resets the per-cycle structured identity.
2. Once `MpcProblem` exists, its current intent, stage geometry, target provenance, horizon and
   schema create `MpccProblemContext`.
3. The actual attempted/accepted formulation updates the structured enum directly. No reason-string
   parsing is used.
4. A solved, finite, constraint-valid and wall-certified result creates
   `CertifiedMpccSolution` before its first control is exposed.
5. A retained Dynamic Escape execution stores and restores the original context and certificate.
6. The node resolves the existing `FinalControlSource` exactly as before, then creates one
   `FinalControlDecision` for logging.
7. The final trace includes context, solution, authority class, identity status and canonical-contract
   status.

## Fingerprint policy

- Use a fixed 64-bit FNV-1a implementation with explicit field framing.
- Hash integral values in a defined byte order.
- Hash floating-point stage geometry by IEEE-754 bits after canonicalizing negative zero.
- Do not use `std::hash`; its cross-platform stability is not a contract.
- A zero fingerprint means unavailable/invalid and is never a valid identity.

## Compatibility mapping

Current formulations are represented explicitly during migration:

- legacy spatial MPC;
- three-state progress-contouring MPCC;
- five-state velocity-progress MPCC;
- low-speed direct control;
- solver-derived hold/fallback bypass.

Only a solved solver formulation can create `CertifiedMpccSolution`. Current direct and hold paths
remain behaviorally unchanged but appear as `legacy-normal-bypass` in telemetry, making their later
deletion measurable.

## Files

- New: `include/multi_purpose_mpc_ros/mpcc_execution_contract.hpp`
- New: `src/mpcc_execution_contract.cpp`
- New: `test/test_mpcc_execution_contract.cpp`
- Update: `CMakeLists.txt`
- Update: `overtake_execution_orchestrator.hpp/.cpp`
- Update: `mpc_controller_cpp.cpp`

## Rollback

The rollback point is `1ecfe78`. Because this slice must not affect control behavior, any command
delta, source-precedence delta, material callback regression, or unexplained incomplete identity is a
rollback condition.

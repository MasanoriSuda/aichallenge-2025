# MPCC engineering rules

Repository-root AGENTS.md applies. These rules add the controller-specific contracts.

## Normal authority

The current normal formulation is the canonical nine-state rate-resolved MPCC.
Track, Cruise, Follow, Hold, Stop, ShiftOut, Pass, Return and Rejoin are intents/constraints.
Target selection, homotopy, map processing and supervision may remain separate components.

- A normal command requires a solved, finite, constraint-valid, physically certified trajectory.
- Solution, executed trajectory, certificate and final command share an immutable problem fingerprint;
  lateral and longitudinal commands share one solution ID.
- Async adoption requires compatible observation, target, geometry, horizon, bounds and cost context.
  Age or target ID alone is insufficient. Incompatible intent/schema/geometry changes invalidate warm starts.
- Compare objectives only with the same horizon, state/input schema, weights, constraints and terminal meaning.
- Shadow-only, schema-only, unattempted and uncertified candidates cannot execute.
- Solve failure permits only bounded, same-formulation, same-context last-certified execution, then
  Emergency Stop. Do not restore legacy MPC, three/five-state or low-speed direct normal fallback.
- Emergency may override normal control. Stuck/contact/gear/reverse Recovery is a separate supervisor
  and retains the upstream normal decision/failure identity.

## Task scope and causal evidence

For an audit/review/plan request, inspect and report without production changes.
For a fix/implementation request, investigate and implement within the authorised scope without
requesting a second approval just because the investigation finished. Past steering approval gates
are history; use the current request and existing session authorisation.

Before a production fix, establish the earliest broken invariant, its producer, a failing test/replay,
downstream masking behaviour, obsolete paths to remove, acceptance criteria and rollback commit.
When evidence is missing, obtain the minimum observation or deterministic reproduction needed first.
An observation-only result cannot become a production candidate merely because it certifies offline.

Repair the producer, remove obsolete masks/bypasses, then run focused and package tests, build,
available replay and dynamic acceptance appropriate to the change. Inspect final authority telemetry.
Use the root-cause-auditor skill for unresolved controller regressions; routine approved edits need no full audit.

Iterations, tolerances, weights, margins, timeout/hysteresis/rates, retry/hold/grace/clamp/fallback,
diagnostic suppression and accepting a failed test are not root-cause fixes on their own.
Any deliberately temporary exception needs causal evidence, explicit authorisation, an expiry and a deletion test.
Parameter tuning follows structural acceptance; do not lower safety or certificate requirements.

## Architecture comparison gate

Keep single certified normal authority invariant; Mission representation, candidate generation,
formulation, convexification schedule and solver backend are replaceable hypotheses.
Before a third patch in one failure family, compare architectures on a sealed snapshot. Do this earlier if:

- two causal hypotheses for the scene were falsified;
- ShiftOut repeats without positive Pass/Return acceptance;
- another resume/reconnect/replenish/retry/timeout/lease/grace/fallback rule is proposed;
- numerical solve and exact physical proof repeatedly disagree;
- fresh/retained authority repeatedly vanishes for the same immutable world/problem.

During comparison, keep production authority and runtime parameters unchanged. Compare:

| Arm | Method |
|---|---|
| A | Persistent Mission with current nine-state SQP |
| B | Stateless receding ManeuverBundle with the same SQP |
| C | Independent rough spline/polynomial/lattice with the same refinement |
| D | Bounded offline multi-SQP or nonlinear feasibility solve |

Share world, state, reference, wall, peers, model and hard constraints; reseal changed candidate identities.
All-method failure is `Unknown` without a bounded physical infeasibility certificate.
Record accepted, rejected and inconclusive outcomes in `docs/spec/mpcc-experiment-registry.json`;
repeat a rejected experiment only when its recorded revisit condition is met.

## Evidence and completion

For each structural change, use one steering with invariant, scope, failing replay/test, changed files,
added/deleted paths, remaining authorities, safety/timing evidence and rollback commit.
A new normal authority requires retiring the replaced one in the same slice; an authorised shadow
measurement instead needs a named promotion/deletion boundary.

Use file:line for code, commit IDs for history and run/Domain/time/decision IDs for runtime claims.
Separate root cause, contributor, mask, detection gap and Recovery behaviour. Label hypotheses and unknowns.
Simulation/SIL/HIL/vehicle results are distinct. A local fix passing does not establish integrated race acceptance.

# Slice 2: Track/Cruise canonical MPCC shadow

## Baseline

- Branch: `develop_july`
- Baseline commit: `156a07f6c98eee47b448d868a03c6926d8eebe3d`
- Accepted predecessor: `.steering/20260822-mpcc-canonical-contracts/`
- Parent migration: `.steering/20260822-mpcc-single-authority-audit/`
- Preserved unrelated working-tree change: `aichallenge/result-summary.json`

## Repaired invariant

Track and Cruise must be representable by the same five-state velocity-progress formulation that is
the migration target, using the same stage geometry and a physical wall certificate, before that
formulation is allowed to own normal commands.

## Earliest pre-slice failure

`config/config.yaml` enables progress-contouring MPCC but limits it to overtake. In
`MPC::init_problem()`, progress metadata is created only when the live formulation is requested.
Consequently a normal Track/Cruise cycle cannot even construct the canonical five-state problem for
shadow evaluation. The visible legacy-normal-bypass trace is therefore produced upstream of solver
selection, not by an OSQP failure.

## Scope

1. Separate availability of canonical progress metadata from live formulation authority.
2. Construct Track/Cruise five-state problems from the exact production snapshot and stage geometry.
3. Solve them with a dedicated shadow-only persistent OSQP context.
4. Reset shadow warm starts on incompatible intent, schema, horizon, or stage geometry; retain them
   across a compatible rolling horizon.
5. Physically validate the shadow trajectory against the current wall bounds and swept footprint.
6. Report solve timing, feasibility, certification, prediction difference, command difference, warm
   starts, resets, and coverage.

## Non-scope

- Do not change the published command, normal authority, source precedence, weights, bounds, or
  runtime parameters.
- Do not flip `progress_contouring_mpcc_overtake_only`.
- Do not share the shadow solver workspace with the live or left/right branch solvers.
- Do not update the live circuit breaker, reentry gate, last-certified solution, retained execution,
  or command post-processing from a shadow result.
- Do not add a fallback, retry, hold, lease, cooldown, or long-lived feature flag.
- Do not promote Track/Cruise authority; that is Slice 3.

## Acceptance

- Existing Track/Cruise output remains `legacy-normal-bypass` in Slice 2.
- Eligible normal cycles have an attempted shadow solve whenever progress metadata is valid.
- A shadow candidate is counted certified only when solved, finite, constraint-valid, converted to
  the canonical prediction layout, and physically wall-valid.
- Shadow results are never executable and never populate `last_solution_contract_`.
- Warm-start compatibility has deterministic tests for same geometry, rolling geometry, intent,
  schema, horizon, and discontinuity changes.
- Focused tests and `make autoware-build` pass.
- A fixed single-car run and repeated simulation show no non-finite shadow result, stale adoption,
  attributable consecutive 25 ms callback overrun, or production command/source change.

## Coverage threshold

For a controllable Track/Cruise interval after startup and before shutdown:

- metadata build coverage: at least 99% of eligible cycles;
- shadow solve attempt coverage: at least 99% of metadata-valid cycles;
- finite solved coverage: at least 95% of attempted cycles;
- physically certified coverage: at least 95% of attempted cycles.

Any lower result blocks Slice 3. It is investigated as a formulation/contract defect, not tuned around.

## Rollback

Rollback commit: `156a07f6c98eee47b448d868a03c6926d8eebe3d`.

Rollback if published command values/source precedence change, shadow state contaminates a live
solver, or callback timing causes stale command publication.

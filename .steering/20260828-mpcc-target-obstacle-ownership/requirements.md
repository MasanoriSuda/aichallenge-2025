# Requirements: keep the active target inside MPCC

## Objective

Make the canonical seven-state MPCC, rather than an upstream Mission-path
certificate, own the active opponent constraint throughout `ShiftOut` and
`Pass`.

## Frozen evidence

- Baseline: `25ee2485`
- Run: `output/20260828-135832`
- Frozen failure: decision `4834`, sequence `3931`, intent `Pass`, target
  `d2`, side `-1`
- The problem identity still names `d2`, while the captured lower problem has
  `dynamic_obstacle_refinement_active: false` and no obstacle stages.
- The current replay world contains a fresh `d2` observation with matching
  generation.
- Rebuilding the same-side candidate from that immutable world solves the QP;
  a physical-diagonal same-side candidate passes exact wall, timed-obstacle
  and terminal-successor proof.
- The upper comparison log under `.steering/ano` keeps an `opp[...]` relation
  in its receding GMPCC output instead of releasing the opponent when an
  upstream path is separated.

## Constraints

- Do not change production authority, solver settings, clearance, speed,
  timeout, lease or fallback behavior.
- Do not weaken exact wall or dynamic-obstacle proof.
- Do not add a case-specific Pass resume rule.
- Keep the upstream exclusion certificate for Mission/orchestrator identity;
  it may not remove the obstacle from a different trajectory owner.

## Definition of done

- `ShiftOut` and `Pass` keep a complete stage-corridor or current-target tube
  in the canonical QP whenever one is available.
- An upstream Mission-path exclusion certificate cannot disable that tube.
- Cruise/Follow behavior is unchanged.
- Focused contract and source-structure tests pass.
- Full package build/tests pass.
- The frozen snapshot remains the dynamic acceptance reference; a new run
  must show a non-empty dynamic contract for equivalent Pass decisions.

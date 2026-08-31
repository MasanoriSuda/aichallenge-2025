# Design: Return canonical target producer contract

## Causal chain

1. Pass requests Return for more than three seconds.
2. `build_prospective_return_problem()` builds a current-world Return problem.
3. `resolve_dynamic_obstacle_contract()` excludes Return from every supported
   interaction intent, so it publishes no dynamic target stages.
4. `build_bounded_candidates()` calls `resolve_canonical_target_horizon()`;
   Return is rejected before solver or physical proof.
5. Atomic phase handoff correctly retains the last certified Pass.
6. The retained Pass eventually loses terminal/wall feasibility and decision
   1839 falls to Emergency Stop.

The visible steering/wall failure is therefore downstream. A/B/C/D and the
independent nonlinear oracle show that decision 1839 is already physically too
late; changing steering or solver settings there would only mask the producer
contract defect.

## Repair

Treat Return as an opponent-interaction intent in the canonical dynamic
obstacle contract resolver. Return receives `CurrentTargetTube` when and only
when that current-world tube is complete. It does not receive the
`StageCorridor` shortcut, because that owner represents the selected passing
side and is valid only for ShiftOut/Pass.

The existing stateless Return builder remains the single consumer. It uses the
tube to classify stay-ahead/stay-behind topology, generate current-world
rejoin geometry, and retain exact ReplayWorld proof. No second prediction,
retention path or fallback is added.

## Alternatives rejected

- Removing the Return target requirement: unsafe for a merge beside the
  target and contradicts the existing topology tests.
- Reusing a Pass stage corridor: stale homotopy geometry is not a current-world
  Return certificate.
- Holding Pass longer or relaxing wall/solver constraints: operates after the
  earliest violated invariant.

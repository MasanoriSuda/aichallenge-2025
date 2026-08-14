# Requirements

## Purpose

Prepare the `DynamicMissionWait` runtime path for the next reachability fix without changing
overtake tuning, safety thresholds, transition priority or generated commands.

## Scope

- Local code around the `update_overtake_line()` replan/paused-Mission authority chain.
- Separate forward-prefix publication from DynamicMissionWait state resolution.
- Flatten DynamicMissionWait action dispatch into one explicit executor.
- Preserve existing replacement priority and the current runtime admission condition.

## Constraints

- Do not change YAML parameters.
- Do not make the forward prefix reachable from tactical rolling replan yet.
- Do not change wall, target-overlap, emergency-brake or solver hard guards.
- Preserve all user changes already present in the worktree.

## Definition of Done

- The forward-prefix publisher and DynamicMissionWait action executor have explicit interfaces.
- Existing behavior remains unchanged, including the currently identified reachability defect.
- `make autoware-build`, package tests and `git diff --check` pass.

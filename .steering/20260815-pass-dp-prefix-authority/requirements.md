# Requirements

## Background

The `20260815-010313` run produced the first recent clean
`Pass -> Return -> Idle`, so the neutral-gap lifecycle correction is accepted.
However, all rolling DP refresh logs still reported
`source=complete_mission`.  MPCC-lite shadow logs showed feasible current-state
prefixes, but those prefixes were evaluated after the behavior output used by
the execution refresh had already been populated.  Difficult attempts
therefore kept an old DP path for several seconds.

A runtime wall center contraction also replaces the frozen Mission.  The
replacement changes the lateral goal, but the copied candidate can carry a DP
path generated for the old goal or fall back to a non-DP lateral profile.

## Goal

Make the freshest physically validated current-state same-side DP prefix the
normal Pass execution reference, while preserving atomic fallback to a valid
complete-Mission DP path and rebasing wall-center contraction onto its newly
validated lateral profile.

## Constraints

- Keep exact target, same-side, prediction-freshness and refresh-interval gates.
- Never replace a complete Mission merely because a DP prefix exists.
- A rejected or stale prefix must not block a valid complete-Mission refresh.
- Runtime wall contraction must not reuse a path aimed at the pre-contraction
  lateral goal.
- Actual wall contact, target overlap, emergency front risk and solver hard
  failure remain fail-closed.
- Do not change ROS topics, messages, services, launch entry points or result
  schemas.
- Preserve the user's existing `aichallenge/result-summary.json` change.

## Definition of Done

- The locked-side shadow assessment publishes its current-state DP prefix
  after shadow evaluation.
- Rolling refresh tries the prefix first and falls back to the complete Mission
  when the prefix is not admissible in the current cycle.
- Wall-center contraction installs a current-state lateral path matching the
  contracted goal.
- Unit tests cover construction and rejection of the contracted transition
  path.
- The package builds and focused tests pass.

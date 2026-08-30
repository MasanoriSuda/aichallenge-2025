# Requirements: current-world Gate A lifecycle

## Objective

Remove the pre-entry lifecycle defect where a current-world executable
dynamic-obstacle trajectory exists, but Overtake admission waits for a second
asynchronous tactical Mission producer until scalar front-risk authority stops
the vehicle.

## Frozen failure

Run: `output/20260831-022355/d1/autoware.log`

- Decision 4569 selected a complete Overtake Mission for target `d2`, released
  the generic Follow cap and reported `validated mission owns longitudinal
  entry`.
- The normal seven-state population then solved and published the
  `normal-avoidance-positive` candidate with exact wall and dynamic-obstacle
  proof (`minimum dynamic clearance = 0.380908 m`).
- `OvertakeLine` nevertheless remained `Idle` because Gate A required the
  separately scheduled dual-branch Mission hint.
- Before that producer supplied a joinable hint, the front distance contracted
  from about 15 m to 12 m and authority changed to `SafetyBrake`; subsequent
  Stop authority reduced speed from about 8.5 m/s to 2.5 m/s.

## Architecture escape-hatch classification

| Path | Frozen result | Interpretation |
|---|---|---|
| A: persistent Mission + Gate A | Mission selected, Gate A absent, no ShiftOut | fails |
| B: stateless current-world ManeuverBundle + same seven-state SQP | solved, exact wall/dynamic proof accepted and published | succeeds |
| C: rough Mission/lattice + refinement | complete geometric Mission already exists; no evidence that geometry is the blocker | not the limiting stage |
| D: offline multi-SQP/NLP | not required because the live seven-state solve already succeeds | not the limiting stage |

Exit classification: **A fails, B succeeds: Mission lifecycle/scheduling
defect**. This is not physical infeasibility, a solver-tolerance issue or a
clearance-tuning issue.

## Constraints

- Do not change speed, clearance, solver tolerance, timeout, lease or grace
  parameters.
- Do not add another normal controller, Mission resume rule or fallback.
- Current-world tactical geometry may enter Gate A only through the existing
  causal prospective seven-state solve and exact physical certificate.
- A stale asynchronous dual-branch result may remain advisory but must not be
  a mandatory producer for initial Overtake admission.
- Active same-side/cross-side replacement continues to use its existing
  no-return and current-world checks.

## Definition of done

- A pure unit test reproduces the inactive pre-entry case: a current-world
  Mission exists while the asynchronous dual hint is absent.
- The resolver selects that current-world Mission as the canonical pre-entry
  tactical input, with an immutable current decision sequence.
- Gate A still performs a fresh prospective seven-state solve and exact
  wall/opponent certificate before phase mutation or command publication.
- Tests reject an absent/invalid current-world Mission and preserve active
  replacement precedence.
- Full package tests pass.
- A short `make dev2` run shows Gate A/ShiftOut can start before scalar
  SafetyBrake in the frozen first-catch class without introducing wall
  Recovery.

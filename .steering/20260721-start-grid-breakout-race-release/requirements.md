# Requirements

## Goal

Prevent a validated start-grid breakout from behaving like Follow before the first hairpin.

## Functional requirements

- Once the existing inflated-vehicle gap check and execution-zone check accept a start-grid side,
  expose the domain/global race speed reference immediately. Do not wait for the internal
  `ShiftOut -> Pass` phase transition.
- Keep the existing MPC acceleration bounds, domain/global speed cap, wall bounds, explicit
  forbidden waypoint policy, and gap-loss fallback.
- Latch the center of the selected, inflated ego-center corridor as the OvertakeLine lateral goal
  at pass entry. A turning front target must not move that goal toward either corridor edge on
  every V2X update. Use the target-relative goal only when no valid corridor center is available.
- Evaluate both inflated side corridors at an unlocked start-grid entry. If both are feasible and
  the front target has a visible lateral stagger, prefer the side away from that target before
  comparing corridor width. A single feasible side still wins.
- If the selected side is rejected by the target-ordering guard before lateral clearance, clear
  only the start-grid side latch so the next eligible entry can evaluate both sides again.
- Ordinary and start-grid overtakes must use the same validated corridor-center goal while
  retaining their staged closing-speed limits.
- While an explicit line owns the lateral reference, refresh V2X obstacle corridor bounds every
  cycle. Reference ownership must not disable obstacle constraints.
- Before lateral separation is established, if the ahead locked target reaches/crosses the
  selected-side ordering, release ShiftOut/uncommitted Pass into bounded Recovery. Hold only a
  transient live-corridor dropout for the configured bounded gap-loss window in this phase. Once
  Pass has latched lateral separation, treat both rotating-frame ordering and live-corridor loss as
  diagnostic-only and let rear-clear plus the explicit hard guards own completion/cancellation.
- Supply a heading-error reference consistent with the explicit lateral offset profile.
- Keep the physical 1.45 m combined kart width, reduce only the fixed prediction/wall buffers,
  and require the explicit pass line plus lateral-clear latch to approach that same width.
- Do not change ROS topics, messages, services, Domain layout, or evaluation result contracts.

## Acceptance

- Unit tests prove that an unvalidated breakout retains the front cap and a validated breakout
  receives the full race reference.
- Unit tests prove that the validated corridor center is computed correctly and a fixed pass goal
  is unchanged when the target lateral position moves.
- Unit tests cover successful and intruding selected-side ordering and the offset-line heading
  reference.
- Unit tests cover target-stagger side preference, its deadband, and gap-availability precedence.
- Runtime acceptance requires a visually confirmed physical pass; course-progress ordering and
  internal Return completion alone are not sufficient.
- `make autoware-build` succeeds.
- The package test suite succeeds.
- In the next `make dev3`, P1/P2 should log `grid_breakout=1`, `speed_cap=0`, `cap_release=1` from
  breakout entry and a finite stable `corridor_goal` while ShiftOut/Pass remains active.

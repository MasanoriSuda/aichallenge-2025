# Task list

- [x] Compare the external review against the latest dev3 logs.
- [x] Isolate the remaining behavior-layer start-grid speed cap.
- [x] Add a tested start-grid breakout speed-reference resolver.
- [x] Latch the initial breakout lateral goal in OvertakeLine.
- [x] Keep the validated breakout line after transient hairpin gap re-evaluation.
- [x] Use ego-relative lateral separation for common-course front overlap.
- [x] Keep breakout race speed after the target becomes side-by-side.
- [x] Keep live gap-planner corridor validation under explicit-line ownership without forcing its
  pass-side interval as an unreachable hard bound during ShiftOut/Pass.
- [x] Abort continuity when the locked target enters the selected-side ordering.
- [x] Before lateral separation is established, hold transient live execution-corridor loss for
  the bounded gap-loss window, then move persistent loss into Recovery.
- [x] Make live-corridor loss diagnostic-only after Pass has latched lateral separation.
- [x] Limit target-side ordering cancellation to the pre-lateral-clearance phase.
- [x] Prefer the feasible start-grid side away from a visibly staggered front target.
- [x] Clear a start-grid side latch rejected before lateral clearance.
- [x] Align the pass-line target and lateral-clear latch with the physical kart width.
- [x] Drive OvertakeLine toward the center of the validated vehicle-to-wall ego corridor.
- [ ] Confirm P2 physically passes P3 in video; projected ordering alone is insufficient.
- [x] Add a geometrically consistent OvertakeLine heading-error reference.
- [x] Add regression tests for immediate release and fixed lateral goal.
- [x] Run `make autoware-build`.
- [x] Run the `multi_purpose_mpc_ros` test suite.
- [x] Record verification results and next-run log criteria.
- [x] Re-run build and package tests after corridor-center line ownership.

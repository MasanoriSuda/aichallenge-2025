# Design

## Evidence

The 2026-07-21 23:09 dev3 logs show that P1 and P2 entered `start-grid breakout` within 0.04 s of
Start and that OvertakeLine's duplicate front cap was already released. The behavior layer still
reported `speed_cap=1` until `ShiftOut -> Pass`; P1 never completed that transition. P2 released
the cap after about 2.1 s, then its target-relative line moved from roughly -1.2 m to -2.4 m as the
front kart entered the first hairpin. The selected corridor then collapsed and behavior returned to
Follow.

## Longitudinal policy

Use the existing `should_preserve_breakout_line` validation as the ownership boundary. If the
breakout is active, behavior is Overtake, the inflated side gap is available, and the execution
zone permits it, resolve the speed reference as a released Pass-stage reference immediately.
Otherwise retain the existing ShiftOut front cap. The result is still capped by `cfg.v_max` and the
MPC's acceleration and dynamic bounds.

## Lateral policy

For each candidate side, use the first active free interval returned by the gap planner after target
vehicle inflation and wall-clearance adjustment. Its midpoint is the usable ego-center corridor
center. When a validated pass first starts ShiftOut, store that center in `OvertakeLineState` and
reuse it through ShiftOut and Pass. This applies to ordinary and start-grid overtakes, preventing
the explicit line from being attracted to a moving front kart. Per-horizon wall clipping and
lateral-acceleration limiting remain active. Resetting OvertakeLine clears the fixed goal. If no
valid corridor midpoint is available at entry, retain the existing target-relative goal as a
fallback.

## Scope

The implementation is confined to `multi_purpose_mpc_ros` behavior/OvertakeLine helpers and tests.
The Ready-time static target latch proposed in the external review is a separate robustness change;
the latest run proved that entry latching was not the reproduced failure, so it is not mixed into
this experiment.

## Initial multi-vehicle side arbitration

`output/20260722-064048` proved that the preceding P2 continuity corrections work: D2 held the
right Pass, changed the locked-target longitudinal ordering between WP105 and WP108, entered Return
at WP125, and completed the line without Follow or a solver failure. The remaining visible
absorption was D1 following D2. D2 was already about 0.14 m to D1's right, but the unlocked entry
selected the wider right corridor. The target-ordering guard rejected that line about 25 ms later,
and the latched side made D1 retry the same conflict repeatedly.

Gap availability remains authoritative. When both inflated corridors are feasible, an unlocked
start-grid entry now prefers the side opposite the visible front-target stagger. If the target is
inside the configured deadband, corridor width and the existing geometric fallback retain their
roles. A pre-lateral-clearance ordering rejection clears the start-grid side latch, allowing the
next eligible entry to assess both sides instead of cycling the rejected side.

## Dynamic execution corridor correction

The 2026-07-22 early-morning run showed that D2 completed its pass while D1 followed the same
right-side command into D2. At the failure, D1 was near `e_y=-1.79 m` and the locked D2 projection
was near `-2.14 m`: the target had entered the selected/right side of ego. The old validated
breakout continuity returned before re-evaluating that ordering. In addition, explicit-line
ownership disabled the runtime gap planner completely, so target `e_y` was clipped only by walls.

The explicit line remains the only source of lateral references, while the live gap planner checks
that the selected-side corridor still exists. During ShiftOut/Pass, a feasible pass-side interval
is not forced into the MPC state bounds: the selected free interval is discontinuous across the
inflated obstacle and cannot represent a reachable transition. The first run showed failure before
ShiftOut moved; the next run showed the same target jump immediately after Pass latched lateral
separation. The live planner keeps a separate last-valid-corridor timestamp. A transient infeasible
result retains the explicit line for `active_gap_loss_hold_sec`, without extending its own deadline;
before lateral separation is established, persistent infeasibility requests Recovery. Once Pass has
latched the configured lateral separation, the same rotating-frame live-planner result is diagnostic
only. Before lateral separation is established, target-side intrusion remains authoritative;
wall clipping, position jumps, explicit forbidden waypoints, cooldown, EmergencyBrake, and solver
guards remain authoritative throughout. Common-course target-minus-ego lateral ordering is checked
before the breakout/uncommitted continuity latch; crossing the selected side requests Recovery.

The 2026-07-22 06:31 run proved that treating target-side ordering as authoritative after the
lateral-separation latch has the same rotating-frame defect. D2 held the right Pass through the
live-corridor dropout, but at waypoint 73 the ordering guard changed behavior from Overtake to
Cruise (`no relevant vehicle`) and immediately forced `Pass -> Recovery`; Follow then pulled D2
back behind D3. Target-side ordering therefore remains an entry/ShiftOut guard only. After Pass
has latched lateral separation, rear-clear confirmation owns normal completion while target loss,
position jump, wall, emergency, solver, and explicit forbidden-waypoint guards still cancel the
maneuver.

The line derives `e_psi` from its offset profile using
`atan2(d'(s), 1 - kappa*d)`. This removes the former conflict between a changing `e_y` reference
and a constant zero heading-error reference during ShiftOut/Return.

## Physical lateral-separation alignment

Video review overrides the former log-only interpretation of `output/20260722-064048`: D2's
projected longitudinal ordering changed and the FSM completed Return, but P2 did not physically
pass P3. The controller was allowed to latch lateral clearance at 1.15 m while its V2X obstacle
envelope was 1.55 m before covariance. Its start-grid goal was also based on a 1.2 m base offset
and only 0.75 m target-relative minimum separation.

The next A/B keeps the 1.45 m combined physical kart width rather than hiding the inconsistency by
shrinking the vehicle model. It reduces prediction margin from 0.10 m to 0.05 m and wall clearance
from 0.80 m to 0.72 m, raises the target-relative pass-line separation to 1.40 m, and raises the
lateral-clear latch to 1.35 m. This remains an aggressive simulation setting, but the explicit
line and state transition now approximate the same physical envelope. Runtime success requires
video-confirmed body clearance and longitudinal passage.

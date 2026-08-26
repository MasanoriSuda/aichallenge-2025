# Audit

## Observed symptom

The clean Track/Cruise run `output/20260827-012442` completed a 43.320 s first
lap, then produced a 76.203 s second lap. At decision 2304, while the vehicle
was still travelling at about 7.73 m/s, normal authority disappeared for one
cycle and Emergency Stop was selected. The next cycle recovered normal
authority, followed by another short authority loss at decision 2308. Wall,
stall and Recovery events occurred downstream.

## Evidence before the repair

- retained revalidation rejected with `continuation-rejected` and the exact
  nonlinear trajectory reason `invalid-lateral-bounds`;
- control-origin position/yaw join errors were only about 0.132 m / 0.035 rad;
- command steering and velocity were reachable;
- callback overrun was absent before the first authority loss;
- static-wall prefix accounting already reported
  `wall_scope=full:75/current_stage:6`, with no current-stage wall rejection;
- serialization joins were exact after comparing the actual float32 wire
  representation instead of a physical command against its calibrated wire
  value.

## Root cause

Retained revalidation replayed the whole remaining open-loop suffix from the
current physical state and treated that whole suffix as one indivisible
authority unit. A later nonlinear sample could leave its lateral corridor even
when every sample in the current executable stage remained valid. The
execution contract then also required `executable_control_stage_count` to equal
the full cursor remainder. Consequently "the successor must be replanned" was
misrepresented as "there is no safe command now", creating an artificial
normal-authority hole and Emergency alternation.

Static-wall prefix validation had exposed the same architectural boundary one
step later in the pipeline. Fixing only static-wall rejection therefore did
not close the producer/consumer contract.

## Repair

- make retained proof scope explicit as `FullSuffix` or
  `CurrentStagePrefix`;
- when only a later nonlinear continuation sample is invalid, truncate and
  revalidate the exact current-stage trajectory;
- propagate the proven stage count through retained proof, production adapter
  and execution authority;
- require fresh current-decision candidates to retain a complete horizon;
- permit finite-prefix authority only for current-world-revalidated retained
  artifacts;
- truncate trajectory and stage-end traces to the same proof boundary;
- leave current-stage wall, dynamics, actuator, dynamic-obstacle and Follow
  failures fail-closed;
- expose proof scope, exact continuation reason and proven stage count in the
  transition and aggregate decision logs.

## Rejected alternatives

- increasing wall margin, changing solver settings or reducing speed would
  tune a downstream symptom;
- adding a timeout, grace period or a legacy command producer would hide the
  missing authority transaction;
- accepting the whole invalid suffix would publish evidence that was never
  proven;
- accepting a prefix for a fresh artifact would conceal an incomplete solve
  rather than bridge a revalidated receding-horizon transaction.

## Static verification

- failure-first nonlinear suffix/current-stage test: passed;
- retained finite-prefix authority test: passed;
- fresh incomplete-horizon rejection test: passed;
- physical adapter, retained revalidation and production adapter focused
  suites: passed;
- full package suite after rebuild: 1,886 tests, zero errors/failures/skips.

Direct test executables initially loaded stale shared libraries from
`/aichallenge/workspace/install`. The authoritative result uses `colcon test`;
focused direct runs require
`/aichallenge/install/multi_purpose_mpc_ros/lib` first in `LD_LIBRARY_PATH`.

## Dynamic acceptance still required

The next clean Track/Cruise Gate must show that a later-suffix
`invalid-lateral-bounds` event produces a one-stage retained transaction, not
Emergency authority alternation. The run must also confirm zero serialization
join rejection, zero continuous callback overrun, no current-stage wall
acceptance, and recovery by a successor solve on the following cycle.

## Post-repair dynamic evidence

`output/20260827-020001` exercised Track/Cruise for six recorded laps. The
previous `continuation-rejected` authority alternation did not recur,
serialization rejection remained zero, and no callback overrun initiated the
failure. The run therefore accepts the retained-prefix repair itself.

The complete integration Gate does not pass. Laps were 43.235, 40.619,
75.263, 61.994, 88.971 and 60.679 s. Starting on the third lap, the kart
repeatedly lost several metres per second within one odometry sample in the
same course region. This is a separate physical-event defect, not evidence
against the prefix contract.

The first event was recorded at bag time 98.128 s and decision 4445:

- measured speed changed from 8.738 to 5.013 m/s in 0.0190 s;
- the preceding serialized command still requested 8.707 m/s and
  +1.366 m/s2, so MPCC/Emergency braking did not initiate the loss;
- the vehicle velocity report was already near 0.134 m/s while filtered
  odometry still reported 5.013 m/s;
- localization acceleration reported about -19.2 m/s2;
- occupancy sampling reported `map_sample=clear`, `map_contacts=0`;
- only the following decision rejected the old artifact as
  `velocity-unreachable` and selected Emergency Stop.

Four further first impulses occurred at 173.590, 235.803, 324.454 and
385.196 s around x=89615--89618, y=43165--43166. Every preceding command used
+1.366 m/s2. One event included an IMU impulse over 900 m/s2. The evidence
therefore places the first cause outside longitudinal command arbitration and
inside the physical path/AWSIM collider boundary. `analyze_speed_collapse.py`
reproduces this correlation from the bag without modifying production code.

The next root-cause slice must compare clean and impacted passages through
this complete curve, including predicted/actual footprint and AWSIM-visible
collision geometry. Increasing a wall margin, lowering speed, or retaining an
old artifact after a physical impulse is not an accepted repair without that
causal evidence.

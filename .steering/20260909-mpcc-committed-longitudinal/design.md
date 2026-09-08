# Committed longitudinal input and latency prediction

Continue from observation/audit commit 03e174e5. Production control is unchanged
at this point; full completion remains open. Prior executed Stop453/failure993
is sealed in ../20260909-mpcc-published-stop-audit/results.md.

Earliest demonstrated producer mismatch: the complete current-to-control-origin
path holds measured acceleration while already serialized command changes enter
the same declared interval. On the sealed failing state, exact native reproof
rejects and a causal committed-input rollout passes. Original physical artifact
integration agrees with independent Cartesian ODE within 0.003488 mm. Existing
A/B/C/D and fresh Stop on unchanged state all reject, without physical
infeasibility proof. Do not add retry/replenishment or weaken hard proof.

Before implementation, compare temporal models at multiple saved same-run
observations, including acceleration and Stop transition. Distinguish applied
command time, measurement/filter time, and control origin. The known AWSIM
longitudinal input is latest-value at 10 Hz; the existing 0.13 s prediction origin
is a controller model, not an identified exact longitudinal application phase.
Compare observed-response hold, absolute committed controls at existing origins,
and measured-response plus committed command changes. Do not fit a new delay,
weight or acceptance tolerance to this event. Future measurements are scoring
only; forbid their use in predictor inputs.

Implementation boundary after comparison: successfully serialized commands are
the input owner. Define causal history pruning, duplicate timestamps and backward
clock/reset handling; preserve existing bootstrap/missing-observation authority
contracts. Pose, velocity and yaw-response must come from the same piecewise
rollout. No solver/terminal wall relaxation, second normal authority, or change to
wire topic/type. Review Boost and reverse/Recovery semantics before promoting.
Native failing temporal regression precedes producer edits. Full build/package
checks and sealed replay precede new single/dev2 dynamic acceptance. Production
edits/builds and simulations never overlap. Rollback: 03e174e5.


## Selected structural implementation

Absolute command replacement is rejected: full single moving-observation MAE
0.1655 m/s versus the previous0.0558; replacing the measured response would
lose drag/grip/steady-speed behavior. A response delta against only the current
command also fails after a brake already entered the measured acceleration
filter. Pair the velocity derivative and integrated command input through the
same filter, then predict using known future command + their filtered residual.
The0.9gain is carried from the existing twist2accel observation law, not fitted.
Both filters now live in one observer driven by the controller's own odometry
callback. This removes the separate acceleration subscription and its unmatched
callback/filter epoch. The external localization/acceleration producer/topic is
unchanged for other consumers. Current-source repeated timestamps do not invent
a derivative duration or advance the filter; update the speed anchor. Reset/gaps
require a new positive-time observation before prediction is available.

Use the existing odom_timeout_sec both for stale observation rejection and
history retention. Preserve the active command before that valid history window
and later transitions. Backward publisher clocks reset the observer; equal stamps
replace the queued value. Only successfully serialized normal/failsafe command
accelerations enter history. No newly solved command is a predictor input.
The physical-steering interpolation and0.13s default prediction interval remain
unchanged; a piecewise acceleration rollout provides one common pose/speed/
response/path. Model magnitude is used for the separate reverse supervisor's
observations; no reverse normal authority is added. Boost is an unmodeled response
seen by the paired observer; explicit integrated Boost/Recovery coverage remains
required and is not established by the ordinary replay alone.

Native pre-fix call reproduces1.9137333m/s where the declared committed-input
and measured-response model gives1.4204765m/s. New native tests23/23pass,
including nine temporal/steady-drag/duplicate/reset/gap/stop cases. Native replay
at recorded publication receipt order: dev2 moving MAE0.12364→0.07443m/s,
single moving0.05727→0.05700m/s. Decision993velocityerror0.45943→-0.03382m/s.
Bag callback ordering differs from the controller's receive queue; invalid/future
source rows are excluded from this offline metric and cannot become acceptance.
Paired full-path physical proof and dynamic trials are still required.


## First integrated rejection and clock producer repair

`20260909-committed-longitudinal-single-r1` rejects at D1decision1938,
7.6238m/s. The new paired filter is not the rejecting physical predicate: no
wall proof is reached. Cached local ROS time is35.444999207s; the already
received odometry is stamped35.449999207s (received1788900176.9714034,
command1788900176.9715824). The producer chose the older cache time, so the
strict prediction guard correctly refused to bind it to a later observation.
Native receive-order replay reproduces unavailable at this exact command.

Keep the guard. Align the entire control decision, prediction, proof and final
publication to the later of the local ROS cache and the already received
observation in the same epoch. Retain the exact original integer ROS stamp;
do not add tolerance or shorten delay. Independent /clock and odometry delivery
is not a future physical observation. Source-clock skew outside the existing
odom_timeout_sec remains rejected. A regressed local ROS clock invalidates the
old odometry and observer, requiring a new observation pair in that epoch.
Native temporal tests now27/27pass (four added clock-owner cases). This changes
the source clock producer, not the failed predictor guard. New full build,
package tests and single-r2 are required. The rejected r1 and its original
source/shared-library/config manifest remain immutable.

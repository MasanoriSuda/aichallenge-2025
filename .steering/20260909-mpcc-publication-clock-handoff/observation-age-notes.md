# Next causal audit: observation epoch alignment

These are read-only findings during the clock-handoff runtime campaign, not a
production change or selected solution. Use sealed EKF-repaired world 937,
not this currently running trial until it is stopped and analyzed.

- Exact 936 prefix[0] equals Odometry stamp 9.884999779; Request.now 9.899999778.
- Exact 937 prefix[0] equals Odometry stamp 9.904999778; Request.now 9.929999778.
- mpc_controller_cpp.cpp control() resolves common epoch using max(received
  Odometry stamp, ROS clock), but begins its fixed 0.13 s latency prediction
  with the original Odometry pose and speed. Prediction intervals begin at the
  later control clock. Thus the raw state can be relabelled by 15/25 ms.
- The canonical prefix, predicted pose, current speed and obstacle field use
  this later now. Raw wall/recovery observations have separate responsibilities.
- Steering resolution uses steady receipt ages for sensor/committed command
  and ROS control age for publication reachability. Its source timestamp and
  inferred yaw response need explicit causal alignment too; do not move only
  one pose stamp and claim all physical state epochs are consistent.
- For 937 the raw steering report at 9.904999778 is 0.1219261661 rad;
  next report at 9.939999777 is 0.1599231660 rad. Request current physical
  steering is 0.1357111998. Future report is diagnostic only, never a live input.

Before repair: native/replay the actual controller prefix production from
same-run measured inputs and committed command history, preserving all source
and receipt clocks. Quantify which state components are at each epoch, then
select a consistent causal alignment and test constant motion, acceleration,
turn, duplicate/future/cache-behind/reset cases without relaxing source-age
limits or adding fallback/retuning. The existing constant-turn-rate predictor
may provide a bounded observation-motion model, but its adequacy for joint
velocity/physical steering/yaw-response is not yet established. Do not simply
set every decision to Odometry time (can repeat/backtrack committed clocks).

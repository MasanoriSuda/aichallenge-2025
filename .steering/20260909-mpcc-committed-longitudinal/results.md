# Longitudinal response producer results

Baseline/rollback:03e174e5. The implementation is locally validated; integrated
completion remains open. All positive, rejected and inconclusive outcomes stay
in the validation ledger and central registry.

## Causal chain and selected implementation

Actual published Stop453and failure993are the immutable basis from the prior
audit. Holding measured acceleration throughout the existing0.13s interval
ignores already serialized braking. Native original trajectory integration and
same-world wall-proof rejection are both reproduced without re-solving.

Absolute command replacement is rejected across the old clean single run:
moving MAE0.1655m/s versus observed hold0.0558. The chosen observer instead pairs
velocity derivatives and time-integrated committed input through the same0.9
filter, retaining their response residual. Exact command switch intervals then
produce one pose, speed, yaw response and physical path. No future observation
or unexecuted candidate supplies a predictor input. Existing timeout and delay
are unchanged; no solver/margin/tolerance/weight/fallback tuning is included.

This removes the separate MPCC acceleration subscription and its unmatched
filter callback. The external acceleration topic remains for other consumers.
Duplicate source times refresh the velocity anchor without advancing either
filter. Backward clocks, invalid data and observation gaps reset or invalidate
the observation; history retains only the existing valid age window and pending
command transitions. Native temporal tests cover these contracts and stopping
without reversal. Reverse Recovery stays a separate supervisor; explicit Boost
and Recovery dynamic coverage is not established by ordinary forward replay.

Native receive-order replay scores moving dev2MAE0.123640→0.074430m/s and
clean-single0.057266→0.056996m/s. At failure993, velocity error changes from
+0.459432to−0.033825m/s. Bag reception differs from the controller callback
queue, so invalid/future-source rows are excluded and these scores are not
exact live replay. Future velocity reports score outputs only.

Actual Stop453/world993physical replay uses the prediction shared library,
predicts1.420476467m/s and passes the first stopping reference. Old recorded
state rejects in the same report. The separate successor remains
course-frame-unavailable; no new successor rule was added. Physical steering
precision and non-seam wrap surrogate limitations from the prior replay remain.

## Runtime outcomes and clock repair

- Single-r1: rejected at D1decision1938,7.623776m/s. Local clock35.444999207s
  trails already received odometry35.449999207s by5ms. Strict observer guard
  rejects before wall proof. Native receive-order replay reproduces it. No
  result-details/penalty acceptance. Before shutdown39callback windows,
  1581cycles,max13.455ms,0overruns. Original source/config/shared DSO are sealed.
- Clock repair: preserve that strict guard; choose the later of the local ROS
  cache and the already received source stamp for the entire decision, proof
  and publication. Keep its original integer timestamp. Local clock regression
  clears old observation/history; skew beyond existing odometry age is invalid.
  Four new native clock-owner cases pass.
- Single-r2: startup-inconclusive. AWSIM stays Spawned; its clock publisher is
  discovered but no clock samples or controller odometry arrive. Unity log and
  external-stop reason are preserved. Runner shutdown restores protected JSON.
  Exact native Unity/DDS wait cause remains unknown. A fresh simulator process
  meets the recorded revisit condition for unchanged-code single-r3.
- Single-r3: six laps253.768981934s,penalty0, no active moving override or
  non-NormalControl Recovery. Reported258callback windows/10461cycles,
  max16.564ms/0overruns.10424reported final actuation joins/0rejections and
  10459/10459canonical production commands; these are throttled window totals,
  not per-message certificate replay.10458recorded control commands at
  39.999938Hz, maximum receipt gap37.079096ms,0gaps>50ms. One observer-bootstrap
  refusal is Spawned at zero speed, outside Ready/Start.
  Source stamps still include22duplicates/backward and two>50ms gaps
  (maximum100ms). Same-run /clock and sensor receipt gaps reach~300ms; source
  clock steps stay5ms. This delivery/stall cause is unresolved, not a repaired
  timing defect. Existing odom age guard is unchanged. Same-run results are
  copied at Finish because the runner originally polled the post-collection
  run path; its future completion detection now checks observed Finish and a
  source JSON modification time after launch. User JSON is restored exactly.
- Dev2-r1: rejected, D1decision4428at8.42m/s and wall1788902492.091281413.
  Source3834retention loses terminal wall contingency; delay/control path stays
  clear and the separate Stop successor is static-path-blocked. D1has one lap;
  no race result-details/penalty acceptance exists in the sealed run. Explicit
  shutdown starts1788902501.7472665; the odometry/clock errors after1788902503
  belong to teardown, not the initiating failure. D1reported3972callback cycles,
  max15.974ms/0overruns; D2reported3970cycles,max28.758ms/1overrun. Both control
  topics stay~40Hz with0receipt gaps>50ms. User JSON is restored exactly.
  First-bucket snapshot deduplication captured startup source296but omitted
  actual source3834at4428. The separate terminal failure snapshot exists with
  its current world. Do not reconstruct3834from296or another re-solved plan.
  Next: compare A/B/C/D on this new sealed world, repair the bounded observation
  bundle so every written failure carries its corresponding source/artifact,
  then obtain the minimum missing live evidence before a further control fix.
  Repeated solve/exact-wall disagreements keep the architecture gate active.

## Validation and limits

Native test suite27/27passes. `make autoware-build` builds25packages in3m42s.
`colcon test --packages-select multi_purpose_mpc_ros` passes all60CTestgroups;
`colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`
reports2359records,0errors/0failures/0skips. The earlier whole-workspace result
aggregation encountered stale unrelated package.xml files; its2490total is not
the MPCC count and the traceback is retained. Scoped earlier observer count2355
and clock-repair count2359are distinguished. Syntax/YAML and git whitespace
checks pass. Pre-commit is not installed on this host; no hook pass is claimed.

Same-run dynamic authority and bag timing remain required. Complete peer Follow,
both signed Pass/Return, multi-tick moving Stop/restart, separate Recovery/Rejoin,
async identity, dev3/dev4, gates and tar/baked-image evaluation remain open.
The exact10Hz simulator longitudinal phase has not been identified;0.13s is the
existing prediction model. These local repairs do not establish full acceptance.

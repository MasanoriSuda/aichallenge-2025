# Validation ledger

- Baseline commit03e174e5; paired-response observer candidate remains uncommitted.
- Native old production call fails saved committed-input temporal invariant
  (1.9137333 versus1.4204765m/s); preserved in
  output/20260909-committed-longitudinal-before/.
- Initial native suite23/23pass; initial25-package build passes3m54s.
- Initial package60CTestgroups pass. Whole-workspace colcon test-result scans
  stale unrelated package.xml paths and reports2490records; this is not the
  MPCC count. Scoped result explicitly2355records/0errors/0fails/0skips.
  The aggregation traceback is retained, not hidden or counted as a test pass.
- Native actualStop453/world993 uses the built prediction shared library,
  reconstructs the observer from received commands/odom (no future input),
  predicts1.4204765m/s and certifies the first terminal Stop reference. All
  old recorded-state rejections remain visible in the same report.
- Single-r1integrated trial fails D1decision1938at7.6238m/s: source observation
  leads the local ROS clock cache by5ms. No wall proof is reached. Callback
  reported39windows/1581cycles,max13.455ms/0overruns before shutdown. No race
  result-details or penalty acceptance. User JSON restored byte-for-byte.
- Native receive-order replay reproduces the clock rejection. The new epoch
  resolver preserves the strict predictor guard and anchors the whole control
  decision to the newer already-received source timestamp. Clock rollback and
  out-of-contract skew remain invalid. Updated native suite27/27pass.
- Epoch repair: second full build passes all25packages in3m42s. All60CTest
  groups pass; package-scoped result2359records/0errors/0failures/0skips.
- Single-r2never enters Ready/Start: AWSIM remains Spawned and no controller
  odometry arrives. Domain1discovery sees awsim_d1 clock publisher with reliable
  QoS, but10-12s one-shot subscriptions (reliable and best-effort) receive no
  clock. Unity main thread is waiting on futex; GPU is available. The exact
  native Unity/DDS wait cause is unknown. Unity startup log is preserved under
  that run. External interruption uses runner cleanup and restores user JSON.
  This is startup-inconclusive, not driving acceptance or a control rejection.
  Revisit condition: fully terminate that stalled simulator process and launch
  a new one with unchanged production binaries/config in single-r3.
- Single-r3six laps253.768981934s/penalty0; active moving overrides0 and no
  non-NormalControl Recovery.10461reported callback cycles,max16.564ms/0overruns.
 10458bag control messages39.999938Hz,receipt max37.079096ms/0gaps>50ms.
  Source duplicates22and two>50ms gaps remain; /clock and sensor receipt stalls
  reach~300ms while /clock source steps remain5ms. Cause stays unresolved.
  At Finish, same-run JSON is preserved by mtime/state before explicit runner
  interruption; future runner fixes result-path completion detection only.
  Protected user JSON restored byte-for-byte.
- Dev2-r1rejected at D1decision4428,8.42m/s, terminal wall contingency after one
  lap. Actual3834source is absent because separate published-source deduplication
  already captured startup296; world4428is preserved. D1callback3972cycles,
  max15.974ms/0overruns; D2callback3970cycles,max28.758ms/1overrun. Both control
  topics~40Hz/0receipt gaps>50ms. Clock/odometry teardown errors occur after the
  explicit1788902501.7472665shutdown and are not the initiating failure. No
  final race details/penalty acceptance. User JSON restored exactly. Further
  control changes require new-world architecture comparison and missing actual
  publication evidence.

No physical infeasibility, exact simulator longitudinal phase identification,
Boost/Recovery dynamic acceptance, or full-race completion is claimed.

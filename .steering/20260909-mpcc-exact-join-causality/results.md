# Exact join and GNSS heading causality results

Baseline/rollback466bba5e. Full MPCC acceptance remains open. The current repair
is in racing_kart_gnss_poser, not a change to MPCC limits or candidate objectives.

## Rejected hypotheses and bounded comparisons

Same-run world945Request predecessor-0.042629476636648178rad maps through sealed
gain1.435to exactly the previous serialized float32wire-0.061173297464847565rad.
Its20ms age matches source stamps10.069999774→10.089999774. Bag receipts are
separate clocks, and this does not prove perfect delivery or tracking.

All nine normal A/B/C/D/G arms reject at solver. Four support Stop arms:
two solver-reject, two solve but fail exact dynamic proof. Four full-rest
missions (free/max-law × inherited/zero objective) all solver-reject.
An independent45arm physical-control search (five steering targets × three
braking levels × three acceleration prefixes) finds no wall-and-peer-clear stop.
It uses the same exact current physical Request, full peer body, wall reserve,
actuator/yaw model and 5ms midpoint integration. It is only a geometric oracle:
it does not verify all original QP state/progress constraints or issue a solved
certificate. All-method failure remains Unknown, not physical infeasibility.

## First demonstrated upstream producer defect

The V2X source sample at10.049999775putsd2GNSS at
(89631.5859375,43132.6953125). At the same source time, its GNSS poser output
is(89631.846,43132.6954), orientationidentity. Repeated paired source samples
show+0.26m mapX and approximately zero mapY, despite moving vehicle yaw≈2.45rad.
This is the existing antenna-to-base offset applied without a usable heading.

In position-difference mode, GNSSPoser resets a function-static previous position
after every sample. The0.2m heading threshold therefore becomes a per-sample
speed threshold;20Hz low-speed motion never accumulates enough displacement.
The static anchor is also shared by independent node instances. Downstream
imu_gnss_poser leaves the finite identity quaternion and displaced position.

Actual-node baseline tests:4of5fail in983ms. Slow north producesyaw0insteadπ/2
andmapX+0.26instead the rotatedmapY+0.26offset. West/reverse and independent
nodes also fail. Noise preserving an already accepted direction passes.

The repair stores the accepted position anchor as a node member, initializing
it once and advancing it only when a heading observation crosses the existing
threshold. Function-static sharing and unconditional anchor reset are removed.
The same forward/reverse rule rotates the existing extrinsics. No threshold,
covariance, ROS topic, V2X body geometry, MPCC objective or safety condition changes.
Before sufficient motion, position difference still cannot observe stationary
heading. This repair does not claim to solve that separate initial limitation.

## Local verification

`make autoware-build`:25packages/38.6s. Both affected-package test commands
completed. GNSS has30records,0errors/failures,6cppcheck skips; all9native cases
(four covariance and five heading) pass. The ROS cppcheck wrapper explicitly
skips version2.7 for performance issues. A direct host cppcheck2.7run on changed
source and the new test with Google Test definitions completed without findings;
this does not relabel the six wrapper skips. MPCC:60CTestgroups/2371records,
0errors/failures/skips. No new compiler warning match.

Repaired actual-node replay uses the original NavSatFix records and declared
antenna extrinsics: D1 136fixes/136paired outputs, D2 144fixes/143paired outputs.
Last source time10.049999775: old yaw0; new D1yaw2.1672454680/D2yaw2.5287377483.
Position shifts D1(-0.4060441501,+0.2151071970)m and
D2(-0.4726823913,+0.1495533364)m, maximum magnitudes0.4595029467/0.4957770098m.
This reproduces the repaired producer on real inputs. It is not a replay of EKF,
controller state or a demonstration that every past Emergency is now resolved.

## Runtime and completion

Fresh single-r1 finished6laps/254.349090576s/penalty0, observed moving overrides0,
callbackmax14.674ms/0overruns across259windows/10495cycles. Command receipt
40.000378Hz/max34.098148ms/no gap>50ms. Source timestamps12duplicates/backward,
one100ms gap; odometry receiptmax309.045553ms/twogaps>50ms and clock305.557489ms.
These delivery issues remain separate from GNSS heading.

Fixed dev2-r1 rejects at D1decision941, wall1788920835.131485764,
measured speed1.299626668701309m/s. Both terminal and final941snapshots are
preserved with worldfingerprint5417572569565392666; actual and inspected370,
exact Request and all grids are present. No finished result or lap.
Zero-solve replay reproduces terminal-contingency-unavailable: steering join now
passes (previous0.0490533038974, expected0.0664825866694,
bounds[0.0306914081923,0.0674151996024]rad over15ms).
Current continuation wall is clear; terminal peer proof failsd2 at
-0.00120685195265m; independent Stop wall is clear but peer fails
-0.000951979315212m. Alternate371/928was rejected at current join; it is not
substituted for inspected370. Original370wall/dynamic pass separately.
D1/D2callbackmax20.730/23.434ms and0overruns,9windows/367and365cycles.
The new observation reliably preserves both categories at one final failure.
GNSS unit/recorded-input repair is accepted, but coupled acceptance stays rejected.
All runtimes stopped and protected user JSON restored.

Next: bounded architecture/physical controls comparison on exact941/370 and
an audit of the preceding current-world peer/terminal viability transition.
Do not repeat945unchanged or claim GNSS repaired all controller failures.
No compile, production edits or heavy replay runs alongside the simulator.
Record all outcomes and seal input/binary identities, final authority and timing.
After this repair, moving Stop/restart, Follow/both Pass/Return directions,
Recovery/Rejoin/Boost/async, dev3/dev4, gate1/2/3 and same-artifact submission
evaluation remain required. Local commits only; no push.

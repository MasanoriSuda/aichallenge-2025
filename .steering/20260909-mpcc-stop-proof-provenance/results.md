# Stop proof provenance — local repair and rejected coupled acceptance

Baseline `ae6862aa`. Autonomous MPCC completion remains open. This slice repairs
one demonstrated certificate producer defect; it does not establish the cause
of the preceding coupled race failure or promote an offline Stop variant.

## Production change and native evidence

`stop_lattice_shadow::build_wall_snapshot` copied a pre-solve observation's
`bound_tolerance_m` into the physical proof. The strict dynamic validator checks
this against the exact trajectory's artifact-derived lateral residual bound.
Different values cause rejection before any peer samples, explaining why an
empty blocker can accompany `dynamic_valid=false`.

The new native test seals a valid observation baseline `4.2e-5` different from
the solved artifact's tolerance. Before repair it fails with
`direct-seven-state/obstacle=/minimum=inf`; four other current-world Stop tests
pass. After binding the physical snapshot to the exact trajectory's value,
all five tests pass. The observation and its fingerprint remain unchanged.
The existing strict mismatch validator and its regression remain in place.
Stop rejection detail now includes the existing source-validation reason.

No candidate selection, authority, hard bound, solver budget, weight, configured
tolerance, proof acceptance rule or clock changed. Rollback: `ae6862aa`.

- `native_certificate.py before`: 4 pass / 1 demonstrated failure, preserved.
- `native_certificate.py after`: 5 pass.
- `make autoware-build`: 25 packages pass, 15.4 seconds.
- Container `colcon test --packages-select multi_purpose_mpc_ros --event-handlers console_direct+`:
  60 CTest groups pass. Scoped `colcon test-result --test-result-base
  build/multi_purpose_mpc_ros --verbose`: 2369 records, zero errors/failures/skips.
- Existing setuptools deprecation warnings remain; no pre-commit success claim.
- Fixed single/coupled runs are closed; outcomes follow below.

## Previous coupled run: preserve identities and missing observations

Run `20260909-complete-rest-dev2-r1`, D2 first atomic terminal failure is
**decision 923**, fingerprint `8706044921014371466`; its recorded published
normal artifact is **328**. The later visible moving Emergency is **925**.
The selected alternate **395 / source decision 908** is absent from the bundle.
Never substitute a newly solved candidate or the 925 log's steering for these
missing observations.

Exact same-run odometry at the 923 pose gives stamp `10.484999765`, speed
`1.7831378636160988 m/s`, position `(89631.78326342879, 43133.09708785421)`,
yaw `2.0792990617703255`. The 923 current-time physical steering and controller
steady receipt ages are not recorded. Bag receipt times cannot recover them
exactly.

The serialized command chronology has:

| ROS stamp | Wire steering (rad) | Acceleration (m/s²) | Evidence |
|---|---:|---:|---|
| 10.449999766 | -0.3315732181 | 1.329596162 | preceding normal command |
| 10.484999765 | -0.2716494799 | 1.329595923 | aligns with 923 Stop selection |
| 10.499999765 | -0.2956811488 | 1.329596162 | next command |
| 10.534999764 | -0.2694311440 | -3.0 | aligns with 925 Emergency |

Control messages have no solution ID. Selection logs plus this timeline do not
by themselves prove 395's final publication. The 925 retained log independently
identifies normal 328 with publication origin `10.630000`, cursor `2.065000`;
it is consistent with a normal reappearance between selection and Emergency.
The exact switching cause remains unproven.

`replay_original_328.py` rebuilds the **recorded** execution with zero solves.
Original-world wall and dynamic proof pass, minimum peer clearance
`1.1823616922198157 m`, terminal speed `4.1666666423761454 m/s` (not rest).
This is not a current-world join. The first helper attempt failed to link the
certified-plan library; its output is preserved, and r2 adds that library only.

D1's 40-cycle window at wall time `1788913399.102002869` has 3 overruns,
callback max `31.967 ms`, MPC max `8.882 ms`, Recovery safety max `21.954 ms`
(40 full / 0 skipped checks). These are window maxima, not one correlated
call stack. The expensive Recovery safety work is a measured contributor;
its exact overrun cause still needs a per-cycle observation. Startup warnings
and pre-Ready overruns remain separate from moving-race acceptance.

## Sealed-world architecture comparisons

The unchanged A/B/C/D/G comparison on 923 rejects all nine normal arms.
Rough-right C reaches a solved QP but fails physical wall proof at sample 27;
the other arms fail solve. The four existing Stop-support arms accept their
first two inherited-objective maximum-braking candidates and reject both
zero-objective candidates at physical wall proof. The CLI's historical report
title says “free Stop”; actual construction fixes the velocity/braking law.
Physical infeasibility is therefore not established.

The original proof-metadata source-copy probe on scenes 2056, 2162 and 923
rejects at solve before it can compare proof tolerance ownership. Those three
observations are inconclusive for live attribution; no runtime claim is drawn
from the independently failing native fixture.

A second explicit formulation comparison shares the new production's uniform
5-second clock, exact world, full peer QP guidance and physical proofs, with
four separate candidates per frozen scene. It changes objective and/or adds
maximum-braking constraints **only in the copied offline producer**; changed
contexts are resealed. It has no current-observation join, Store or publisher.

| Scene | Free / inherited objective | Free / feasibility objective | Maximum braking / inherited objective | Maximum braking / feasibility objective |
|---|---|---|---|---|
| 923 | solver reject | solver reject | accepted | physical wall reject |
| 1629 | accepted | accepted | solver reject | solver reject |
| 4264 | solver reject | physical wall reject | solver reject | physical wall reject |

923's accepted candidate fingerprint `11443900282038282666`, problem context
`9950622728724821938`, first acceleration `-2.959595970943086 m/s²`, terminal
speed `-7.019534838402031e-12 m/s`, minimum peer clearance `0.664353915140041 m`.
1629's free feasibility candidate fingerprint `12060840357004980841`, context
`3414977626868885795`, first acceleration `-0.6592506327024255 m/s²`, minimum
peer clearance `1.8369943417650998 m`. All accepted source snapshots and complete
execution artifacts are captured, together with all rejected outcomes.
No one variant establishes a cross-scene solution, and 4264 remains Unknown.

## Fixed runtime after the metadata repair

`output/20260909-stop-provenance-single-r1` finishes six laps in
`253.60894775390625 s`, penalty zero, no observed moving Emergency/Recovery.
257 reported callback windows / 10415 cycles, max `15.511 ms`, zero overruns.
10436 commands at `40.000656863 Hz`, receipt max `34.638166428 ms`, zero receipt
gaps above 50 ms. Source time is not clean: 11 duplicate/backward command
stamps, two source gaps above 50 ms (max `114.999998 ms`). Odometry receipt max
`225.143432617 ms`, three gaps above 50 ms; clock receipt max `226.051092148 ms`.
The one causal-observation warning is startup decision 529, state Spawned,
actual speed zero. Race completion passes; observation/source delivery remains
unresolved. Do not report all timing as passed.

`output/20260909-stop-provenance-dev2-r1` is rejected. First visible moving
Emergency is **D1 decision 951**, wall time `1788915787.658695311`, speed
`1.718736 m/s` in the retained log. No lap/finish evidence or result details.
D1/D2 each have nine callback windows / 366 cycles, maxima
`17.646 / 21.194 ms`, zero overruns. Commands are `40.016141113 / 40.001853088 Hz`,
receipt maxima `38.850069046 / 35.659074783 ms`, no receipt gap above 50 ms.
Source stamps have `10 / 11` duplicate/backward values and `1 / 2` gaps above
50 ms. Odometry receipt maxima `295.33624649 / 286.435365677 ms`; each has one
gap above 50 ms. D1 has one zero-speed startup causal warning, D2 none.
No causal claim connects those delivery gaps to the later moving failure yet.

The new atomic first terminal world is **949**, fingerprint
`1008990672511552908`, with actual normal **365**, source fingerprint
`6085679935950841233`. Its publication decision is 943, origin
`9.689999786000001`, artifact cursor `0.23571678418568887`; world observation
`10.229999771`, control origin `10.359999771`. It fails terminal peer proof
against d2 at `-0.00122436 m`; static wall is not subsequently attempted.
949 selects Stop **379**. The 951 retained log names 379 on a PublishedPlan
clock, cursor `0.600000`, first publish `10.360000`, first cursor `0.550000`.
Current predecessor steering is `-0.128003`, expected artifact steering
`-0.010359`, delta `0.117644 rad`; reachable bounds
`[-0.155977,-0.100028]` over `0.015 s`. Terminal peer proof also rejects d2;
independent Stop from source 365 rejects d2 while its wall proof is clear.
Stop 379's complete artifact and exact 951 world are not captured.

Repeated observation gap identified for the next slice: the controller treats
an accepted asynchronous **submission** of a terminal record as though this
decision were already recorded, and skips its generic final-authority record.
The downstream recorder deduplicates terminal records by failure bucket, so
949 can occupy that bucket and discard 951 before Emergency clears the
execution ledger. Next obtain a bounded observation-only reproduction and
repair this recording boundary; do not infer a missing artifact or add a hold.

Both simulators are stopped. Protected user JSON hashes match their pre-run
values; no runtime output is committed as user-authored input.

## Remaining work

405 artifacts are sealed; the registry contains 75 snapshots / 163 experiments.
Canonical specification/status, helper syntax, registry references, local links
and diff whitespace are checked. Locally commit the repair, then continue the final-authority
recording gap, current-world/actual-published Stop evidence and architecture
selection before the remaining moving Stop/restart, both Pass/Return sides,
Follow, Recovery/Rejoin/Boost/async, dev3/dev4, gates and submission acceptance.

# Receiver scheduling remains an open physical-input contract

Baseline/controller rollback: `cbcda112`; full M1–M6 remains authorised and open.
No production change in this audit. Do not turn a diagnostic delay into a runtime
parameter or relabel a rejected run. Restore/retain the original Unity DLL.

## Current evidence

- Uninstrumented dev2-r15 first D2movingEmergency: decision956, wall1789045203.077537803,
  wp31/1.47m/s, actual published Stop449. It occurs before ShiftOut; the side-owner
  repair has not yet received dynamic coverage. Native exact input loses terminal
  peer clearance at−0.002840729680m; fresh Stop is−0.000425109729m/100samples.
- Captured955request against source447 replays accepted. Its capture status is
  invalid relative to the failure-source449 association; it is not an exact
  replay of449's prior publication. Its own fresh Stop has+0.007748825651m.
- Four zero-observation-prefix diagnostics keep actual449 and compare modeled
  versus current ego, old955 versus current956 peers. Fresh Stop passes when
  either modeled ego or old peer observation is substituted, but fails with
  current ego/current peers. All four retained checks pass after dropping the
  prefix, so these diagnostics are not equivalent to full publication proof.
  Do not use them to declare a single-factor causal fix.
- The latest raw velocity/tire components are20msolder than pose at956. Merely
  propagating them with the same immediate-brake assumption is not supported:
  public velocity rises1.476625 at10.884999756 to1.507032 at10.919999755 and
  1.537267 at10.954999755, then falls. A correction that predicts earlier
  deceleration could enlarge Stop clearance while worsening the actual error.
- A/B/C/D on956: A solver-rejects; B/C/D cannot create a target-bound candidate
  because the front target is unavailable. Two complete-rest QPs reject. This
  is Unknown, not proof that the physical scene is infeasible.

## Actual application observation at the current controller

`application-dev2-r2` uses the previously validated private diagnostic DLL copy,
not the submitted/original DLL. It first fails D2decision1001 at1789046056.994942900,
wp31/1.15m/s, source576Stop. Shutdown starts1789046061.7581284. Input now11.884999734;
current origin12.014999734. Exact native terminal clearance−0.000016848041m;
fresh Stop−0.000167764143m/80samples. Historical1000/source574 remains accepted,
fresh Stop+0.020348976161m/70samples; its failure-source association is invalid.

Receiver D2id1007536790 has507unique public stamp/float signature matches. Every
recorded assignment still matches its selected receive sequence and float input.
D1 mapping has513matches and2minority matches: equal timestamp/acceleration keys
plus a missing recorder packet can appear unique on the other car. The analysis
now labels receiver association as a full-run inference, preserves all counts,
requires a one-to-one dominant mapping, and does not claim per-packet Domain
certainty. Original strict-assert failure is preserved in analysis-r2; corrected
analysis-r3 carries the result. D2association has no minority matches.

Actual D2selection around the first sustained brake:

| event | source/ROS time | value |
|---|---|---|
| positive seq502 selected | source11.709999738 / apply11.764874323 | +1.329597 |
| first negative seq503 received after that assignment | source11.729999737 | −1.25243032 |
| seq504..506 overwritten before next selection | source11.759999737..11.814999735 | −3 |
| seq507 selected | source11.829999735 / apply11.866269907 | −3 |

Receive503 monotonic1760473091946 is after apply502 monotonic1760472913896.
The first negative source-to-application interval is136.270170ms, including
source/publication/receive and queued selection effects. This is an observation,
not an application upper bound. D2all749scoring windows speed MAE improves
0.032854226→0.017255838 with prescribed actual application;30brake-crossing windows
0.197511063→0.040101079m/s. These plant scores include later changed commands and
are not prospective normal-control or race acceptance.

The local DLL CIL explains the separate input paths:
`VehicleRosInput::<Start>b__53_2` sets longitudinal pending/latest under lock,
then directly sets SteerAngleInput. `UpdateQueuedLongitudinalInput` applies the
latest pending acceleration after its configured interval, using Unity Update
and last-applied time. Steering then has its independent100msmechanical delay.
Do not describe the two channels as a single simultaneous application event.
The existing CIL and original DLL hash are preserved with earlier instrumentation.

## Conditional delivery sweeps and earliest remaining boundary

The diagnostic starts with the same captured modeled current body and world.
A recent positive applied input is held until the hypothetical new Stop reaches
the actuator, while the separate steering history is retained. It then proves
the complete Stop and prefix using unchanged wall/peer checks. If delivery is
later than the nominal130msorigin, extend the diagnostic prefix before beginning
Stop; do not silently assume braking at130ms. Samples0/50/100/150/200ms are five
hypotheses, not an exhaustive uncertainty enclosure or production delay bound.

| captured input | immediate Stop clearance | first sampled delivery rejection |
|---|---:|---|
| uninstrumented949/source412 | +0.113162994m |100ms |
| uninstrumented950 | +0.033732547m |50ms |
| uninstrumented955/source447 | +0.007748826m |50ms |
| instrumented994 | +0.112708564m |200ms;150msstill+0.009136449m |
| instrumented995 | +0.095090242m |150ms |
| instrumented1000/source574 | +0.020348976m |50ms |

Zero-delay rows reproduce the separate direct-Stop replay. Later failure956 and
1001 remain rejected at every sampled delivery time. Each row is a conditional
native calculation, not a reconstruction of that run's unknown future application.
Peer observations and control gains/limits are unchanged.

## Next bounded implementation decision

Earliest broken semantic contract: publishing a normal/Stop packet is modeled as
immediate longitudinal application, even though pending latest-value selection
can retain a positive input or overwrite a brief brake. A certificate must bind
the possible applied-input schedule, not just the outgoing serialized packet.
The responsible boundaries are `predict_observed_vehicle`,
`predict_published_history`/`predict_prospective_publication`, physical prefix,
terminal Stop construction/materialization and retained/async proof identity.

A fixed fitted delay cannot represent overwritten pulses and variable selector
phase. A temporary minimum-hold rule does not repair the shared physical model.
Reject neither all alternatives nor declare completion from a better speed MAE.
The next comparison must use an explicit latest-value receiver model with supplied
receipt/Update epochs as an oracle, then a causally available uncertain schedule.
The empirical model authorization does not make an observed maximum a guarantee.
Any bounded profile needs declared assumptions, held-out evidence and rejection
outside that profile. Finite sampled schedules cannot be called a continuous
bound. Common future controls, complete rest and physical sweeps must hold across
all schedules admitted by the chosen model, with one serialized first command.
Evaluate candidate feasibility and computation cost before a production slice;
fix the producer and every proof consumer together. Existing nominal body kernel,
physical margins, solver budget and single normal authority remain unchanged.

Then resume fixedcommit dev2, actual moving Stop/rest/restart, all intents,
repeated single/dev2, dev3/dev4, gates and same-tar image/evaluation. No new user
confirmation is required for that authorised work.

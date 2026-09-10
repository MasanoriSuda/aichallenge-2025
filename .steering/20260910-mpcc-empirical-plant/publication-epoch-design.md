# Separate nominal decision time from observed publication time

Baseline 8bfdaa52. M4–M6 remain open. The exact r12 D2 932/933 pair
loses terminal peer clearance while executing Stop; its actual Unity input
selection is unobserved. Public velocity rises until source 10.359999768
despite a first negative packet stamped 10.274999770, and then falls. This
does not identify that run's application phase or justify a fitted fixed delay.

Independent earliest broken invariant: `record_published_vehicle_command`
stores the callback-entry decision stamp as an already-published input epoch.
In r10 D1 decision 1668, the packet declares 28.284999367 but the bag receives
it between clock 28.434999364 and 28.439999364, after a 164.662143 ms callback.
The next prediction therefore integrates braking over time before publication.
Bag receipt is not controller or Unity application time, but this interval and
the producer identify the backdating defect. Later source observations can mask
the prior input error; they cannot retrospectively make the packet available.

Extract the existing bounded serialized history recorder into the shared
prediction library. Feed it a fresh ROS-clock sample immediately after final
publication, retaining the nominal packet epoch separately at its call boundary.
When the ROS clock cache trails a causally received observation, use that
observation/decision epoch as the lower bound, as the existing clock contract
requires. Repeated epochs retain the last packet; clock regression clears old
history; preserve a predecessor for the existing freshness window. Invalid
records fail atomically. No new application delay, fit, grace, margin, command
authority, solver setting or command payload/stamp change is introduced.

Regression: a brake selected at 1.0 but emitted at 1.15 must not decelerate the
reconstructed body during 1.0–1.15. Compare against explicit native advance
with old input before actual publication and braking afterwards. Keep early,
repeat, regression and invalid-input boundaries. First run this against the
extracted old producer, then repair and run package/build/native comparisons.
Production and failsafe both consume the repaired recorder. The prospective
packet proof retains its explicit nominal decision epoch; actual future
receiver/application timing remains empirical model error and is not closed by
this history fix. Fresh uninstrumented simulator acceptance remains required.

The bounded application diagnostic at this same controller HEAD observes all
627 assignments matching the exact selected receive ID/source/float32 input.
It is a different run and first fails D2 1294 after ShiftOut→FollowPrepare→Idle.
It does not reproduce r12. Direct observed application, used solely as a posthoc
prescribed input, reduces D2 148 brake-crossing speed MAE from 0.068265 to
0.024975 m/s with the body model unchanged. D1 has no moving brake-crossing
scoring window. This supports a separate receiver-schedule modelling issue,
not transfer of application phase between runs. No failed campaign is relabelled.

Rollback is the focused local commit for this slice based on 8bfdaa52. Archive
r12 and diagnostic manifests, full results, unavailable previous evidence at
1294 and all rejected/inconclusive architecture arms in the registry.

Validation complete for this local producer: buildr28 26packages/4min48s;
testsr22 2406records/62groups/0errors/failures/skips/28.78s. The unchanged
current-world native1294 reproduces intent-mismatch; actualpublished source778
supports a fully certified Stop when its upstream identity is retained.
A/rightB/C/D/G certify this world, leftarms reject; two direct complete-rest
SQP clocks reject at4000iterations. Therefore physical infeasibility is not the
cause of1294. Separate explicit Stop handoff repair follows before fresh
integration. No application timing model or receiver phase is promoted here.

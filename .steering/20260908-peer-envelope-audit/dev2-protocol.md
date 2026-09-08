# Initial full-peer-body multivehicle feasibility

One initial `make dev2` trial, Domain0AWSIM/manager andDomain1/2Autoware,
with the newly built coordinate model and shared complete peer body.
Use standard dev2spawns/count start/collisions and noNPCs/handicap/wall recovery.
A steering-only simulator mount changes only unlimited laps/timeout to6/600.
This is a workspace development trial, not baked submission acceptance.

Prerequisites: peer build and allpackage regressions pass; the coordinate
initial single run and additional geometry-lineage tests have passed.
The prior20260908three-single gate was for an older failed configuration;
this newly authorised producer repair has its own bounded initial protocol.

Stop on first tactical Recovery after preserving2seconds, both vehicles
finished, or840shostdeadline. If both stay at rest/no lap after start, retain
the run until current failures have complete snapshots; do not shrink body
geometry or bypass the physical check to force departure.
No concurrent build/replay/source change during this run. Seal all results,
including penalty, starvation, missing target, bad frame or timing failures.

Check both vehicles' full ShiftOut/Pass/Return episodes, signed side, actual
final authority, wall/peer proof and immutable fingerprints. Check certified
multi-tick Stop independently: an unobserved Stop remains untested.
Any target-free or blocked-start scenario is not proof of general passing
infeasibility. Compare shape conservatism/observable orientation alternatives
only on the newly sealed world. Do not repeat an unchanged rejected run.

Post-r1monitor correction: future trials also stop at the first logged moving
Emergency/Recovery while AWSIMReady/Start, or an active Stuck recovery action.
The tactical phase alone missed r1'sIdle-phase overrides. The preserved r1
script/manifest retains its original behavior; a new trial requires its own
run ID, passing prerequisites and seal before invoking this helper again.

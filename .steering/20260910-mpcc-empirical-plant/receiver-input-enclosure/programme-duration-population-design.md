# Programme duration population

2026-09-12. Continue authorized M1-M6 without routine confirmation. Baseline and
rollback e7825da8; build107/test93 (26 packages,2584 records/66 groups,C5 160).
The candidate-generation repair is now locally validated below. All M4-M6 remain open.

## Frozen runtime and first incorrect boundary

Standard r65/r66 never reached advancing vehicle clock/state/odometry after
Unity sensor initialization. They are startup-inconclusive, not controller race
failures. Host and temporary-container ptrace were denied; no stack obtained.
Monitor Off was observed and a transient force-on action made it On without
releasing r66. No DPMS timeout/security settings were changed. Rendering remains
a hypothesis; do not infer a precise GPU deadlock from a futex wait alone.

R67 changes only Unity to batchmode/nographics, retaining RViz and original
physics, sensors, controller, timings and constraints. It reaches Ready/Start,
but stays nearly stationary with no laps. This is a renderer diagnostic, not
standard repeated-run acceptance. Complete teardown restored user artifacts.
All recorded commands are finite; source/record frequencies are distinct.

D1 publishes789 positive-acceleration packets among6825; every positive packet
is immediately followed by a nonpositive packet. D2 publishes256 among6293;
its longest positive sequence is10 packets,0.225s between first/last source
stamps. Whole-bag peak measured longitudinal speeds are0.16432/0.16284m/s.
Gear and control-mode topics are absent from the bag: no state is inferred.
Recorded sends are not claimed as actual AWSIM input application.

R401 recreates the original scheduled sources, with2 reserved prior packets and
about70ms lead, from D1admission458, D2admission455 and D2context1539. The normal
source population chooses one positive packet plus braking after its full
source-horizon candidate fails. These Ready admission snapshots include expected
Track-to-Cruise generation revocation; that revocation is not the cause of the
candidate population defect. D2context1539 rejects the new geometry while the
same dispatch accepts its old published source. Do not conflate those contexts.

The earliest defective boundary is the finite candidate population: one positive
interval or the entire remaining original source horizon. Failure of the latter
is not evidence that only one interval can safely accelerate. R402 traces D1's
74-interval proposal to a terminal wall rejection, and D2's185/156-interval
proposals to terminal CourseGeometryUnavailable. The exact D2 geometry subbranch
is not yet established. The producer is evaluate_stop_candidates in
mpcc_rate_resolved_retained_revalidation.cpp. Valid certified short Stop output
is a downstream safe response, not an authority bypass to remove.

## Bounded comparisons

R403 independently reconstructs count-limited constant-input proposals under
exactly the same original source/observed world/model/constraints and complete
pending-input/full-rest proof. Counts2,3,5,10,20,40 pass all three scenes;80
passes both D2 scenes; each complete original horizon still fails. New contents
receive new proof identities. The diagnostic count environment variable exists
only in generated offline code; it is not a runtime configuration proposal.
Three positive packets need12.8/15.6/11.4ms source proof offline, not a live bound.

R404 performs existing A/B/C/D comparisons on the embedded original solver
source in each failure transaction. All arms in each comparison share that
source's immutable world/state/reference/model/hard constraints. Persistent A
passes all three. For the two Track sources tactical B/C/D are candidate-rejected;
for the Cruise source B/C/D right pass and left reject at solver. This upstream
solver comparison is explicitly a different temporal boundary from scheduled
C5 conversion; it cannot replace R401-R403's complete applied-input proof or
prove live adoption. It gives no reason to replace Mission or the solver in
order to fix this downstream duration-population omission. No infeasibility
claim follows from rejected candidates.

## Intended repair and verification boundary

Compare producer repair with mask removal and wholesale representation changes:
- Removing the short certified Stop would discard a valid safety response.
- Replacing the upstream Mission/solver does not repair C5's two-endpoint duration
  population; the sealed upstream source already solves and certifies.
- Add one independently proved intermediate duration derived from already
  reserved appointments, while retaining original hard constraints and source
  identities. This targets the observed missing candidate directly.

For a reserved job, the next-source planning span is prior-count plus one
original publication interval. After a full source-horizon proposal fails, try
that positive duration (bounded by the immutable remaining source horizon), then
retain the existing short certified candidate path. Pending2 means3 intervals;
this is not a tuned count or new rate/timeout parameter. Zero-prior bootstrap
ordering remains as specified. Each proposal independently proves all original
pending memory, complete stopping response, current world, source velocity
ceiling, exact float packet contents and immutable original clock windows.

The count limit must stay private to candidate generation. A supplied count or
elapsed budget must not mint a proof. Changing programme contents creates new
input fingerprints; a new candidate cannot inherit actual suffix history of an
old different source. No normal authority is added. Retain complete current
membership/geometry/generation/history and before/after25ms checks.

Required before promotion: a small native failing fixture for feasible middle
versus rejected full horizon; positive-prefix/history and current world/clock
negatives; full package/build; original-source replay with explicit changed-source
history rejection; review, sealed evidence and local commit. Then committed
runtime. Additional worker work, source replenishment, steering responsiveness,
actual Stop/rest/restart, all intents and the remaining M4-M6 are unverified.
The standard graphical startup issue remains independent of controller changes.

## Implementation and local results

The count remains a private optional argument of evaluate_with_stop_profile.
A reserved source whose full duration fails now independently proves the prior
count plus one interval. If the original duration is no longer than this count,
the duplicate is skipped. Existing short Stop and zero-prior ordering remain.
No node/YAML/public API or safety/clock values change. Two source files changed:
the retained producer and a peer-stop/native-history regression; no authority
path was removed because the short certified Stop remains valid.

R407 provides the minimal one-prior fixture: three positive packets fail to stop
before the peer, two pass. R408 is the expected RED. R409's helper-epoch failure
is preserved and corrected before R412 passes all161 native tests. R405 missed
a local include and R406 did not yet reproduce missing-middle coverage. R410
used the old pre-build library and is a setup failure, not final validation.
Build108 passes26 packages; tests94 passes2,585 records/66 groups/source105.
R413 reproduces three new three-positive candidates on the recorded runtime
sources, with new hashes and unchanged original appointments; both earlier proof
failures remain. R411 checks12 actual-history cases and original clock guards.
[Sealed evidence](programme-duration-population-evidence.json). Standard r68 and
all remaining M4-M6 acceptance are required; no runtime success is claimed.

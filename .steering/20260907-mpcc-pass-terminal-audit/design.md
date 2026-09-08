# Hypotheses and comparisons

At decision 4166 the retained plan's terminal contingency becomes unavailable.
An independent published Stop shadow has accepted wall proof but dynamic-path-blocked
against d2. A Stop-lattice candidate exists but current-world join rejects steering.
Pass later enters SafetyBrake at decision 4177 and times out in FollowPrepare.

1. Replay A/B/C/D on the frozen current problem and unchanged world.
2. Compare the existing declared-lateral Stop contract arm. This separates terminal
   representation from physical dynamic prediction and async candidate availability.
3. Compare immutable peer prediction with measured future V2X only as diagnostic
   evidence, not as an oracle available to production.
4. Inspect actual candidate build time, accepted controls and current steering
   reachability before proposing a scheduling or contract repair.

The frozen target has velocity (-1.728757,-0.576989) m/s and acceleration
(2.696102,0.242755) m/s². Raw future V2X stays close to constant-acceleration
extrapolation for 0.5 s, but differs by 4.54 m at 2.02 s and 31.88 m at 5.02 s.
That long-horizon prediction error is a contributor hypothesis, not permission to
weaken a certificate or to declare it the first cause before the replay comparison.

The controller is unchanged during this audit. Baseline source/binary/config is
identified by ../20260907-mpcc-semantic-target-time/dev2-manifest.json.

## Confirmed producer defect and repair boundary

The complete A–H comparison and the existing terminal-lateral comparison all
reject. A reconstructed wall-only warm start collides at t=0.505 s, so its
physical oracle also rejects. That early collision is within the measured
short-term prediction agreement; long-term acceleration extrapolation does not
explain every rejection.

Seven independent nonlinear braking probes preserve the declared Stop velocity
law. Six pass the complete wall and timed dynamic proof through rest when the
solved Stop is its own contingency, as in the existing production Stop owner.
The seventh collides at t=0.84 s and correctly rejects. The standard normal
oracle instead synthesizes a different racing-line Stop from the initial state;
its rejection is not a proof that the independently solved Stop is impossible.
The additional Stop oracle remains standalone and cannot publish.

The actual Stop lattice rejects before physical proof: its initial QP has
steering-rate-prefix row 354 value 0.0845271 rad, upper 0.0722918838 rad.
Schedule generation insets per-stage steering rate (rad/s), but never intersects
the independently inset cumulative steering delta (rad). Saturated fixed inputs
therefore contradict the QP's unchanged cumulative rows.

Regression `StopLatticeRatesSatisfyCanonicalCumulativeSteeringRows` compares the
schedule to the actual canonical adapter's rows, both signs and every stage.
It fails before repair. Share the cumulative boundary resolver between adapter
and lattice; intersect rate and cumulative bounds before choosing a fixed input.
Delete the rate-only saturation assumption. No proof/solver tolerance, margin,
population size, authority, lease or retry change. Replay must certify the
repaired Stop before dynamic acceptance. Roll back only these adapter/lattice
changes if disproved; preserve the semantic target-time repair.

## Rounded-corner Stop constraint comparison

After the cumulative steering repair, all 68 existing fixed Stop schedules still
reject on effective-progress obstacle rows. The isolated negative schedule
(sign=-1, switches=2/6) has a nonlinear rollout that passes the exact C++ wall,
timed dynamic and declared-velocity Stop proofs through rest. Its worst obstacle
rows are 495/496: violations 0.155057666/0.148807194 m. The same controls therefore
provide a physical witness excluded by the all-behind convexification.
Evidence: `output/20260907-mpcc-negative-stop-schedule/{comparison.log,
nonlinear-proof.log,nonlinear-primal.json}`. This remains a newly reconstructed
current-problem witness, not the missing historical retained artifact.

Next comparison is offline only: derive each complete separating plane from the
nearest point on the oriented ego rectangle to the peer circle. Keep exact body
support, canonical QP, nonlinear proof, fixed controls, Stop law and world.
The plane is not relaxed to the wall witness. Its orientation is linearized like
the existing physical axis rows. A synthetic corner-clearance regression covers
both sides and rejects a colliding displacement. Unavailable geometry or an
undefined separation normal must reject. Production remains on its current
branch while this comparison runs. Promotion requires a certified QP Stop plus
focused/package tests and a live authority/adoption audit; otherwise keep the
comparison explicitly observation-only and record the rejection.

Call-site qualification: the current controller submits
`EvaluationMode::DirectSevenStateOnly`. Thus the fixed schedule prefix repair
corrects the optional/offline lattice producer, but does not itself alter the
live Stop computation. Do not attribute a live improvement to that repair.
The new physical-support comparison certifies the predeclared -1/2/6 fixed Stop
in 20.545 ms in one offline observation, while the old row set rejects it.
This is not a timing KPI or evidence of production adoption. A separate free
seven-state Stop comparison now tests the actually scheduled formulation.

## Free Stop mission comparison and promotion criteria

The free Stop support QP is affine feasible: independent HiGHS finds a primal
with 5.22e-15 maximum original-row residual and its full C++ nonlinear Stop proof
passes. Alternative internal equilibration, equality condensation and a single
positive objective-unit change do not establish accepted canonical production
solutions; do not promote those numerical changes.

The existing maximum-braking producer fixes speed to zero after 1.5 s but
inherits the moving normal mission's lateral/heading/progress objective through
5 s (terminal progress reference 19.7907 m versus roughly 3.2 m stopping travel).
A four-arm comparison preserves the maximum-braking law and hard constraints:
old objective/axes rejects; old objective/oriented support rejects; feasibility
objective/axes rejects; feasibility objective/oriented support certifies a free
seven-state Stop, including wall/dynamic/full-rest proof, in one 6.635 ms offline
observation. Objectives across these arms are not ranked against each other.

Promote a declared braking-feasibility mission, not a tunable weight set:
remove all inherited racing objective terms in the Stop producer; preserve
references used as numerical seeds and all hard bounds, model, dt and world.
Select exact oriented support only for this complete fixed-velocity-to-rest
feasibility contract. Normal missions retain their existing objectives and
branch semantics. Do not add a new publisher, retry population or runtime flag.
DirectSevenStateOnly remains the live scheduled computation. All downstream
certified-plan storage, current-world join, prefix/terminal proof and Emergency
boundaries remain. Invalid contract/geometry fails closed.

Regression first verifies the canonical assembled Stop has no racing objective
and preserves source bounds/time/world/identity; it fails before the producer
repair. Add classification negatives (moving terminal, incomplete/full-horizon
mismatch, tracking objective, nonmonotone or nonfixed speed) and retain the
geometric regressions. Acceptance requires package tests/build, frozen direct
Stop certification via the production entry, then bounded dev2 with final
current-world adoption evidence. Revert only this Stop objective/support slice
if falsified; base commit a8b968ac2e4a86a28432a52d132774459d355cb5 plus prior semantic
time/prefix fixes is the rollback reference. No solver/tolerance changes.

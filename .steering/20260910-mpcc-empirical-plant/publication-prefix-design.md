# Bind each new command to its own prospective input prefix

Investigation baseline/rollback: `3ab839fa`; diagnostic commit `da5eac7c`.
Continue the existing autonomous M1–M6 authorization. The diagnostic and
following production slice share this causal record.

## First failure and exact reproduction

Run `20260910-nine-state-dev2-r8`, D2decision2144 at1789030367.405416185,
world `2c67b444e00f72e6`, inspected/published Cruise1673. Negative-side
ShiftOut→Pass→Return→Idle completed earlier. Emergency occurs at3.88m/s.
The actual prior2143andcurrent2144Requests with the same immutable1673plan
replay accepted→terminal-contingency-unavailable. Current maximum-braking
Stop rejects peerD1 at1.740000s, clearance−0.000963873m, body position
(89634.9940367633,43163.4035055659); wall proof passes. Evidence:
`output/20260910-nine-state-dev2-r8-replay-r1` and `...-factorial-r1`.

Prior2142materializes and publishes Stop1675. Recorded serialized history:
40.369999097:wire acceleration−2.755402088, steering0.015675601;
40.389999097:wire acceleration+1.047703028, steering−0.024127916.
2143accepts Cruise1673from predicted control-origin speed3.513718701m/s and
publishes its positive acceleration. 2144predicts3.935095433m/s and rejects.
These control-origin speeds differ0.421376732m/s over a30ms decision interval.
They are predictions at different future epochs, not a claim of measured
physical acceleration. Sensor/model error remains separate.

Named zero-solve factorial comparison holds all hard geometry and limits:

| Counterfactual | Normal terminal | Full Stop |
|---|---|---|
| Exact2143 | accepted,+0.970144m | accepted,+1.396669m |
| Exact2144 | rejected,−0.039740m | rejected,−0.000964m |
| 2144with previous peer propagated30ms, preserving remaining CA horizon | rejected | rejected |
| 2144with only prior predicted speed | accepted,+0.706753m | accepted,+1.116936m |
| 2143with only current predicted speed | rejected | rejected |

Top-level2144snapshot retains the original public observation and published
history. Native replay reconstructs its control-origin position and speed with
exact zero error. Adding a hypothetical−3wire command at captured now, with the
existing nominal delays and unchanged steering, yields3.572809757m/s and a
certified Stop with+0.909311m clearance. Under that braking prefix, ordinary
Cruise also certifies but would publish+1.047703025. Replacing the prospective
brake by that acceleration returns the failing3.935095516m/s prediction.
The added input is explicitly counterfactual; it is never labelled published.
The original course progress reference anchor is held and the native builder
projects the changed world pose. This is not integrated acceptance.

Factorial r2 additionally materializes the prospective-braking Stop through
the existing production APIs: bundle available, exact same-request join accepted,
production packet available with speed3.572809757, acceleration−3 and physical
steering−0.01681387797 (the same already serialized steering after the existing
wire gain). Thus this native Stop's first packet agrees with its proposed prefix;
the apparently accepted Cruise packet under that prefix does not agree.

Existing same-world architecture r1: persistentA solves35.180ms, then rejects
terminal peer clearance−0.0121625m. Target-dependent B/C/D/G cannot build without
a current target tube and are inconclusive for this Cruise scene. Current-world
complete-rest short clock hits4000iterations/124.974ms, while the full5sclock
solves24.613ms and certifies native complete rest. This is a feasible offline
witness, not new production authority, and disproves physical infeasibility.

## Broken invariant and producer

`predict_observed_vehicle` calls `predict_published_history` before choosing the
new command. The last serialized acceleration is extended to the future control
origin (nominal acceleration delay0, steering delay0.1s, origin horizon0.13s).
`build_rate_resolved_current_world_request` shares that one predicted state and
prefix among candidates. A candidate can then publish a different acceleration,
invalidating the input schedule used to justify its initial state and prefix.
The same native plant kernel is used; the mismatch is the command schedule.
Current-world revalidation detects the lost viability one cycle later;
Emergency is its safety response. A Stop hold timer or peer-radius reduction
would mask the defect and is not authorized as its repair.

The original shared-kernel migration remains validated, but its publication
schedule contract is now reopened. Do not claim M1–M3 are fully closed while
this causal input binding is unresolved.

## Required comparison and implementation boundary

Before promotion, compare the existing fixed-prefix authority with a prospective
publication bound to the actual candidate wire input on this same sealed world.
Include the persistent/SQP and stateless/same-SQP paths plus bounded independent
Stop/maneuver and nonlinear feasibility arms as applicable. Preserve rejected
and inconclusive outcomes; a causal diagnostic alone cannot execute.

The intended interface separates already published history from a proposed
serialized packet. Its nominal publication epoch, acceleration and steering
must be sealed with the prefix, current body state, trajectory and final command.
Use the actual float wire conversion once. Reuse all public input epochs,
nominal model/delays and full wall/all-peer proof. Never add a candidate to
historical provenance as if already sent. Missing/incompatible provenance fails
closed; old snapshots remain diagnostic with their original semantics.

Normal candidate actuation selection (cursor advancement and reachable steering)
must be shared with prefix construction. Stop/terminal materialization must bind
the Stop packet, not reuse an acceleration prefix from the rejected normal plan.
Any changed world state must be reprojected and resealed in its actual artifact;
an unmodified solver identity cannot label a different executed trajectory.
Async, adopted siblings, stateless bundles, Stop/rest/restart and serialization
must all consume this same binding. Retire the old shared future-input assumption
for production authority in the same implementation slice.

Validation must reject the braking-prefix/accelerating-publication example,
accept only self-consistent native packet/prefix/Stop evidence, and check gain,
float serialization, unequal channel delays, clock reset and async adoption.
Then package build/tests, preserved failure replay, fresh dev2 and the remaining
M4–M6 campaign. No extra hold/retry/grace/rate changes or relaxed proof limits.

Implementation must preserve one shared actuation selection for the normal
artifact cursor (including stage-boundary advancement and reachable steering).
The first input is available before physical continuation; prepare its own
prospective prefix once, then run the existing complete proof. The separate
Stop-successor API already holds the last serialized steering and maximum
braking for its first publisher interval, so that packet can also be fixed
before prediction without iterative steering or a new hold rule. Seal the
binding in evidence and reject any final packet which differs from it.

## Timing result of the preceding visualization repair

R8D1/D2raw2637/2598samples, p95 12.506282/14.605389ms,
p99 15.920998/20.060415ms, max34.872756/34.990976ms, adjacent overruns0/0,
overflow0/0. Observed marker costs0.092ms/931points and0.329ms/940points.
No invalid command or actuation-serialization join rejection. Source-clock
duplicates10/10 and maximum gap80ms need attribution; the aggregate includes
startup/finish and the final incomplete telemetry window is absent. Short-run
timing improvement does not close fixed-campaign timing or stale execution.
Buildr19/testsr15 remain valid for3ab839fa's unchanged production source.

## Production candidate and verification

`predict_prospective_publication` keeps original observation/history separate
from the proposed float packet and runs the shared plant with nominal channel
delays. `prospective_artifact_packet` and retained evaluation share the existing
stage-advance/reachable-steering selector. Every production current-world
Request requires a matching prefix, recomputes physical course progress and
the complete body state/path, and shifts Follow's ego offset without moving
the canonical peer branch. The maximum-braking Stop API binds its own first
packet. Materialized Stop is rejoined through the same normal path. All these
paths converge on the common command candidate, which rejects a different
final wire acceleration/steering or model identity. No new authority, rate,
grace interval, margin, objective or solver budget is introduced.

Retired producer: using the history-only future prefix as the production
candidate's authority input. The history-only predictor remains the causal
initial model/solver observation; execution must reprove its own proposal.
Legacy snapshots keep their explicit original semantics for diagnostics only.
New snapshots retain both original inputs and the separate proposal so replay
reconstructs the exact native prefix and checks it against the saved Request.

Buildr20 succeeds (26packages,4m46s). Testsr16 records2401test entries with one
actual source-layout assertion failing (reported twice by colcon aggregation):
the feedback selector moved into its shared helper. All four new behavior
cases pass. The existing source contract is updated to check the same rejection
and proof ordering at its new location; no behavioral condition is removed.
Float normalization now occurs inside the common predictor, with an independent
two-channel native calculation using the serialized steering value.

Final buildr21 succeeds (26packages,8.82s); testsr17 passes2401entries,
62CTest groups,0errors/failures/skips in29.38s. Existing setuptools deprecation
is the only build stderr. Relevant pre-commit hooks pass (prefix-quality-r1).
Added behavior cases cover delay asymmetry, unmodified history, nonfuture
epochs/reset rejection, actual float conversion, stage advancement, missing
or modified prefix, wrong final packet, Stop materialization and publication.
Existing async/context/intent and full package regressions also pass.

Native toolr19 on sealed r8decision2144 uses the new production binding API,
zero solves. Exact legacy and correctly bound acceleration remain rejected;
the accelerating normal packet under a braking prefix now rejects with
`publication-packet-mismatch`. Correctly bound Stop accepts, materializes,
rejoins and converts to acceleration−3, steering−0.0168138783983, with
clearance0.909311163m; serialization/reconstruction also accepts and preserves
the original historical input count/last epoch. A separately labelled course
projection sensitivity passes at0.909921247m. The runtime model's exact current
waypoint is not separately captured in r8, so neither diagnostic is mislabelled
as a fresh integrated controller run. Evidence: prefix-binding-r1/result.yaml.

Next is fresh dev2-r9 on the committed candidate, then all remaining M4–M6
acceptance. Local regressions and replay do not establish six-lap completion.

# First integration run audit

2026-09-11 JST. c86c7163, dev2-r16, buildr39/testsr31. No race acceptance.
D2first active moving Emergency is798, wall1789062201.921616498,~.12m/s.
First loss is795, wall1789062201.853098696,~.08m/s, actual published124Stop.
793/794built and published complete Stop121/124.795builds125butjoinfails.
The old outer trace printed the ordinary retained intent-mismatch rather than
this Stop join result. Native r72on exact795finds invalid-current-state,
including after the explicit published-source intent helper. r71driver compile
failed from nested main alias; source/log preserved, not a behavioral replay.

Original current signed COMu=-.0023094885982573032m/s, control-origin COMu=0.
The velocity epoch7.664999828is219.999995msolder than pose/decision7.884999823;
IMU/tire epochs remain in the snapshot. Holding these received samples is the
explicit existing empirical reconstruction, not a synchronized observation.
No observation is overwritten/clipped in this audit. Current source124uses
physical tolerance.009113071834541867and nominal fullrest Stop is accepted.

The nine-state body kernel accepts signed observed u and already has native
reverse correction/rest branches. Source-free Stop validates the original
prospective history/path and accepts this very input. The retained join instead
unconditionally imposes a nonnegative constraint on the earlier observed u,
before inspecting its same prospective path, even when the future control-origin
state and all-response Stop meet their unchanged bounds. This is a validator
scope inconsistency. Fix only that earlier observation gate for a request which
requires an exact prospective prefix and the complete applied certificate.
Keep observed u signed, old unprofiled rejection, nonnegative control-origin and
all future state bounds, including existing tolerance, unchanged. A larger
negative observation outside the original applied-state bound must reject.
This does not repair callback latency or claim sensor/model error bounds.

A failing synthetic Stop regression must distinguish original signed source,
future rest, absent certificate and an out-of-bound reverse body. Then exact795
must traverse the real published Stop helper/build/join and retain its source.
Consumer diagnostics must report the actual Stop join reason, preserving the
ordinary retained failure separately. Rollbackc86c7163. No model constants,
margin, solver budget, rate, retry/hold/expiry or tolerance changes.

Timing is a separate observed failure: D1/D2callbackp99=82.1076/50.6592ms,
max176.759833/242.453387ms, adjacentoverrunpairs246/103(includesstartup/teardown).
D2decision793spends195.261msinStoplattice and17.559msinoutput proof. Optimizing
unchanged arithmetic or sharing identical work needs measured before/after
results; these overruns cannot pass by raising25msor dropping candidates.
Remaining M4-M6 stays open.

## Local validation

Corrected signed fixture r3fails the oldjoin; original build/include and derived
fixture failures are retained. Current test and native r75accept the untouched
795/source124published Stop/join with applied certificate, rest7.914999823and
minimumpeer clearance1.0726770723374652m. Missingproof/larger reverse inputs
reject. The ordinary proposedCruise/oldTrack mismatch stays rejected.

Rest-map r74uses fixed actual parameters and checks256000native values:
30000maps24.7415ms(centered)versus2.13781ms(analytic). r73used an empty parameter
object and returned2; r71nested-main compile failure also remains archived.
Source-r3oldr67loader failed on all4files after the problem-context ABI extension;
recompiledr77/source-r4passes all4worlds and374960native values. The full-program
prediction time is6.85–10.91ms; no25msintegrated acceptance is claimed.
Buildr40passes26packages, testsr32passes2427records/62groups/error-failure-skip0.
Next freshdev2-r17then remainingM4–M6. See signed-rest-evidence.json.

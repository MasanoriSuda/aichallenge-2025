# Validated local correction and positive live comparison

The short-horizon factory regression fails before correction and passes after.
Full build:25 packages in4min29s; only existing setuptools deprecation notices
and underlay override warning. Full package:2317 tests,0 errors/failures/skips.
Source ownership checks:104 pass. The first build attempt had a logger/clock
name error in the diagnostic call, corrected to the controller's existing
static steady-clock logging pattern before the successful build.

Source11046 after repair: rejected at construction with maximum-braking Stop
horizon ends before terminal rest. Long source4166 retains accepted candidate
11485209961842629289 (explicit feasibility/support) and8040571064917241556
(production feasibility), including full physical proof. No parameter changes.

Current controller SHA-256:
0c9c095fb75d8d4be3091ee6ed4cde29271464b3d34d5edc65c858d49c930213
Comparison SHA-256:
9bcb744f18e40d810373e02fb3fff7c91e0720d4c1b3ef4147d8eeab5b684826

Observation run: output/20260908-mpcc-normal-path-stop-observation.
Its manifest and binaries are preserved before launch. It uses the original
six-lap single scene and may terminate early after an accepted moving Cruise
counterfactual. No counterfactual is selected for command publication.
Live observation completed: decision9401/sequence8769, control219.729995s,
speed8.131m/s, normal-path profile passes the same current-world evaluator
where zero target fails. Extra evaluation0.446ms. This is one logged paired
success, not proof that every failed alternative can be repaired. At9407,
sequence8770 loses aggregate authority and emits Emergency. The monitor
stopped the named container after the positive observation; shutdown grace
allowed five laps and later Recovery/Cruise traces. No six-lap result JSON
was emitted in this bounded diagnostic. Production selection is still unchanged.
The next slice is ../20260908-mpcc-stop-reference-feasibility/.

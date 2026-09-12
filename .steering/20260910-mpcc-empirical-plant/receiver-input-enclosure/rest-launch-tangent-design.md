# Complete-rest initial trajectory

2026-09-12. Baseline/rollback45a1b18f. Same autonomous M1-M6 authorization.
No new normal authority, speed threshold, runtime parameter or safety exception.

Standard r70 preserves new-geometry source progress on D1 but fails before
publication1529: nominal27.074999439, entry27.079999394, dispatch bracket
27.109999394 exceeds original27.099999439 deadline. Its callback21.360712ms
is below25ms but did not fit this packet's remaining original window. Later
stationary1858 is27.074336ms. These are different boundaries. D2 admits no
certified source and remains stopped. Both failures stay open; no laps accepted.

D2source1/260 initial QPs are numerically linearly feasible (r430 independently
checked residuals1.34e-16/6.07e-17). They fail OSQP convergence at4000iterations.
The connected initial state trajectory uses a legacy steady-speed input reference:
first acceleration1.37 then0, despite starting with u=vy=yaw_rate=0. It coasts
back into the model's rest transition. The captured stage3 acceleration column
contains lateral/lag sensitivities-99.57/391.99m per acceleration unit. This
supports a nonsmooth initialization/convergence contributor; it is not proof
that the native model, derivative step or solver tolerance should change.

R431 changes only nominal acceleration: bounded speed-directed and constant-power
seeds both solve/prove D2sources1/260, but regress the previously accepted moving
D1source350 at physical refinement. R432 full A/B/C/D: D2source260 current all
reject, speed-directed A/B/C/D/G pass; D1source350 current A/B/C/D/G pass,
speed-directed onlyC/D pass. A universal replacement is rejected. Budgets,
physical bounds, clocks and source worlds were held fixed and identities resealed.

Use a forward initial-trajectory seed only when all three immutable body velocity
states are exactly zero and at least one bounded future soft velocity target is
positive. This is the explicit complete-rest state, not a tuned low-speed cutoff.
Every nominal acceleration moves toward the next bounded speed target within the
original inset input limits; every nominal state remains a shared native
transition. Existing moving-body seeds and pure Hold/Stop braking seeds remain.
Objectives and constraints are not rewritten; the seed cannot publish or certify.
There is one solve pipeline, no retry population, added SQP iteration, old-proof
retiming, command clamp or fallback publisher. Ordinary solved/finite/constraint/
physical/applied-program/final-time acceptance remains mandatory.

R433 applies this state-based seed to five exact sources. D2source1/260 now
solve/prove; D1source350 and r70D1source149 keep their original passing outcomes.
Old D2source320 remains rejected, never promoted. R434 native regression fails
the unchanged coasting producer. Acceptance: new native regression plus existing
moving/Stop/physical negative tests, build/package, built-library frozen complete
comparison, committed dev2-r71. D1publication cost/phase requires its own causal
repair; this D2 initialization change cannot close that timing failure or M4-M6.

Final validation: r435 all24 adapter cases; build11226packages/13.5s;
tests982589records/66groups/source105/C5 161, zero errors/failures/skips.
R436 built production library and original source fingerprints: D2source260 and
D1source350 both A/B/C/D/G pass complete candidate/terminal proof. Old D2source320
A still rejects; C/D accept. R70D1 max1.177314m/s/61positive commands1.5s;
D2 positive commands0/max0.007704m/s. Commands39.0085/34.8821Hz by source stamp,
not actual application. User JSON/DLL restoration and empty Docker verified.
[Sealed evidence](rest-launch-tangent-evidence.json). Next committed standardr71;
D1deadline1529 remains separate. Code: `mpcc_rate_resolved_adapter.cpp:636`
initial tangent producer and `test_mpcc_rate_resolved_adapter.cpp` regressions.

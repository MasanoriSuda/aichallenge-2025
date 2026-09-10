# Complete-rest terminal-time repair

Baseline d0fdd338; production control b0478348. Authorized autonomous M1–M6 remains
open. No production peer forecast, physical margin, solver budget or tolerance changes.

Frozen dev2 D1decision938/world4cdc21da9faa0149 loses its terminal witness when
the peer field updates. The current complete-rest producer chooses a five-second
terminal clock regardless of actual stopping dynamics and inherits a racing
objective. This conflates the supported maximum clock with the required terminal
rest time. It is a candidate-generation limitation, not physical infeasibility.

`output/20260910-nine-state-stop-clock-r2` compares original1.7991s, maximum5s and
native straight maximum-braking0.8s hypotheses, with inherited and zero-cost
objectives. Clock identity remains enforced: the selected candidate maximum dt is
tightened to its declared dt, every peer stage is retimed and fingerprints resealed.
At0.8s, OSQP zero-cost yields a complete certified bundle in24.48ms; HiGHS inherited
objective also certifies in60.09ms. At1.7991s and5s neither OSQP alternative certifies.
OSQP inherited0.8s still rejects, and HiGHS zero-cost0.8s is Unknown. No optimum
comparison across different clocks/objectives is claimed. r1nonmaximum comparisons
failed the clock identity guard before solving and were corrected by r2.

Repair candidate generation with two explicit terminal-time hypotheses, removing
the racing objective from complete-rest feasibility. The first uses the shared
native model's straight braking duration, rounded up to a uniform existing model
integration grid and respecting the source dt limits. This duration is a candidate
clock only, never a stopping guarantee. The second retains the source maximum
support clock so rear-peer interactions can use delayed stopping. Deduplicate equal
clocks. The nominal stage clock cannot be shorter than the publication interval. Its exact
float bits enter the bounds schema, keeping different clocks distinct in artifact
identity and warm starts as well as the interaction fingerprint. Each candidate freely optimizes the same nine-state controls, with all
current-world peers, hard limits, exact body rest and physical proofs unchanged.
There is no steering lattice or increased per-solve/SQP budget. Cancellation remains
checked between hypotheses. Live worker and canonical audit consume the same
population. Candidate clocks, zero-cost meaning and identities remain explicit.

Expected side effect: up to two asynchronous solves when the first candidate rejects;
no extra publisher-thread solves. Preserve measured callback/worker timing. Changing
the terminal horizon changes only what is certified through rest; neither clock
promises indefinite collision-free stationary occupancy. Actual Stop/rest/restart
and coupled runs remain required. Do not count a short endpoint as a five-second proof.

Validation: source immutability; exact terminal rest; distinct clocks/fingerprints;
all-peer provenance; rear/side-peer live/audit equivalence; supersession; package
regressions/build; canonical938replay; fresh dev2 followed by M4–M6. Rollback is a
focused revert of this eventual slice, preserving prior model/map/audit commits.

Other comparisons, not promoted: public-future peer diagnostic makes retained938
pass(+0.0189759m), but twelve Cartesian causal forecasts still reject. Simplistic
map projections pass the scene yet do not preserve initial velocity; smooth
Frenet predictions encounter singular offsets and poor availability on frozen
holdouts. Coherent tangent-only normal/Stop seeds still reject; HiGHS power variant
crashed, hence inconclusive. These are contributor/alternative evidence, not fixes.

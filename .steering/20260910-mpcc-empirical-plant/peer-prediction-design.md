# Causal finite-acceleration peer prediction

Production baseline and rollback: `746ec926`. No simulator or compilation is
running during implementation. Keep all peer radii, ego footprint, wall/solver
tolerances, actuation limits and solver budgets unchanged.

The first dev2-r2 failure is D2 decision991. At that state both current-world
Stop clocks, common-primal SQP/backend comparisons and nine independent native
steering/brake/power laws reject. This is Unknown, not physical infeasibility.
The retained predecessor388 was generated at9.879999779 and is revalidated at
11.684999738. Its original CV proof has minimum separation0.3742811752m.
The same original trajectory and source observation, using its already observed
Cartesian acceleration, rejects at1.9786142s (CA) or2.0374029s (CA for1s then CV).
No future observation or ego trajectory substitution is used by this comparison.
Evidence: `output/20260910-nine-state-dev2-r2-source-peer-r1`.

The producer defect is that the tracker records acceleration but the physical
QP/proof discard it and assume CV over the entire horizon. Current-world
revalidation detects the shrinking clearance late; Emergency is detection and
safety response, not its origin. Ego model error and asynchronous solve latency
remain contributors to investigate if the repaired run fails.

Frozen paired holdout: `output/20260910-nine-state-peer-holdout-ca1-r1`.
The151/61-anchor long runs improve0.5s MAE0.249/0.242→0.121/0.122m and2s
MAE2.665/2.703→1.673/1.758m. The five-anchor short run worsens1s and2s;
long-horizon errors remain large (5s MAE11.053/11.334m). This is an empirical
prediction, never a guaranteed motion envelope. Full5s constant acceleration
and map-based predictions were compared earlier and are not promoted.

Use p(t)=p0+v0*t+a*(h*t-h*h/2), h=min(t,1s), and continuous velocity
v(t)=v0+a*h. Share the function across primary/all-peer QP prediction, Stop
retiming, exact dynamic proof, retained/async proof and diagnostic node checks.
Bound subdivision motion with the maximum endpoint speed over each interval;
the norm of affine velocity is convex, including the acceleration cutoff.
Seal the model/horizon in production problem schema and recorded world hash.
Old snapshots explicitly remain CV (missing horizon means0), preserving their
original fingerprints and replay meaning. No change to public V2X interfaces.

Validation: continuity/invalid-input/accelerating crossing tests, original
source388 replay, snapshot roundtrip/mutation/old-CV identity, package build and
tests, fresh dev2. A passing offline comparison alone is not integrated success.
Also add bounded per-cycle runtime samples to existing observation-only telemetry
so run-level p95/p99 can be computed without substituting window averages.

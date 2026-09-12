# Native nine-state numerical owner

2026-09-12 JST. Baseline/rollback `bc4dded9` (production `56692749`).
Authorized causal repair; all M4–M6 and integrated acceptance remain open.

The nine-state Cartesian body/tire formulation inherited the initial/dynamic
solver policy selected for the older seven-state formulation. The old Follow5575
counterexample has207variables; current horizon20 problems have249variables and
different native body/tire dynamics. The generic RowToleranceNormalized policy
and its historical callers must retain their behavior. A global OSQP toggle is
not proposed.

R492 compares exact original QP matrices, objective, variable coordinates and
warm starts. Row-only normalization stalls at4000iterations on positive2025,
moving3382 and negative2025. The existing row-normalized internal equilibration
solves all six cold/warm cases in275–725iterations, without changing tolerances
or iteration budgets. Startup5 also solves in200/225iterations. These are QP
results, not physical certificates. The positive2025 fixture is a newly sealed
positive-side candidate in the same r76 stationary world; it is not the original
negative-side source2025 QP.

R493 re-solves two already certified current nine-state sources with identical
source fingerprints and no changed side, topology, map, seed or constraints.
Both policies pass the entire physical chain for r72 Follow673 and r76 Cruise3454.
Internal equilibration takes31.100/28.171ms versus40.328/102.146ms in the standalone
replay. These timings are not live scheduling bounds.

Change only the numerical owner in the canonical nine-state SolverContext:
retain separate initial/dynamic and wall/coupled sparse workspaces, but select
the existing row-normalized internally equilibrated policy for both before
any solve. Keep all original physical-coordinate checks, warm-start provenance,
solver limits and final wall/peer/Stop/current/publication guards. Remove the
old row-only ownership of native nine-state initial/dynamic QPs; do not add a
reject-then-retry branch or alter the generic policy implementation.

The failed solver remains correctly unpublishable. Its downstream symptom is
an absent replacement source, then the bounded certified remainder and Emergency.
The R76current3999 wall rejection itself is correct and must continue to reject
its same saved unsafe input abstraction after this repair.

R485–R490 compared native braking seeds and candidate topology. R490 found a
free-input moving candidate with the original wall representation, a braking
tangent, equilibration and a newly selected topology. Its116.730ms source cost
and live adoption remain unresolved. The seed and topology changes are not part
of this slice. Full-pose plane prototypes R487/R488 are not needed for these
successful candidates and have no production promotion. Prepared R491 was not
executed after R490 made that further prototype unnecessary.

Validation and completion criteria:

- [x] Exact failing current nine-state QPs and unchanged physical-row acceptance.
- [x] Current certified Follow/Cruise source regression comparison.
- [x] R494 frozen numerical fixture/generic solver29 pass; source ownership106 pass.
- [x] Build117 all26 packages; package104 all2599 records, zero errors/failures/skips.
- [x] R495 same history/source/domain and exact current3999 rejection retained; R497 built production Follow/Cruise source proofs pass.
- [x] Standard dev2-r77 executed on1241e0cf; rejected at Ready D1final deadline934, no laps. [Current audit](r77-publication-audit.md).
- [ ] Remaining candidate generation, M4–M6 and same-artifact submission/eval.

[Sealed numerical and verification evidence](native-nine-state-numerical-evidence.json).

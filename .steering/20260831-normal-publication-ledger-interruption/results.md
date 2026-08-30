# Results: normal publication ledger interruption

## Baseline failure

`output/20260831-024939/d1` reached `ShiftOut -> Pass` for target `d3`.
Sequence 680 was current-world certified at decision 1279, but the wire output
then alternated between normal Pass and external Stop. At decision 1297 the
store advanced its `PublishedPlan` cursor to 0.45 s although the skipped normal
commands had not been published. Expected steering was about 0.3665 rad and
actual steering about 0.0409 rad. The join was rejected and the later visible
failure was `actual footprint wall margin violated`.

## Architecture classification

The decision-1269 frozen snapshot was evaluated with the repository A/B/C/D
comparison executable. Persistent Mission A, stateless Bundle B, rough/lattice
C and bounded offline D all failed to produce a certified Bundle from the
already-diverged state. The live system had produced a certified Stop successor
at decision 1279 and lost correctness only after publication was interrupted.

Classification: **offline succeeds before the interruption, live continuation
fails after authority alternation: scheduling/lifecycle defect**.

This result does not support changing wall clearance, solver tolerance,
timeout, grace, lease, velocity policy or normal fallback.

## Implemented invariant

`record_final_published_authority()` now clears the store's executed plan and
published-Bundle execution clocks whenever Stop or an actual publisher override
crosses the wire. A separately certified candidate remains data-only and must
pass the existing current-world join before normal authority can return.

The change is at the actual publication boundary. It does not infer execution
from a proposed intent or solver result.

## Verification

- Focused C++ store test: 1 passed.
- Focused single-authority source-contract test: 1 passed.
- `make autoware-build`: 25 packages completed.
- Full `multi_purpose_mpc_ros` result: 2,196 tests, 0 errors, 0 failures.
- `git diff --check`: passed.

Dynamic acceptance run: `output/20260831-031354` (`make dev3`).

- D1 observed four normal-ledger interruptions.
- D3 observed two normal-ledger interruptions.
- A pre-interruption `published-plan` sequence was never resumed as the same
  sequence after an interruption.
- D1 sequence 712 was interrupted and the next logged evaluation was new
  sequence 740 with `clock=bootstrap-candidate`.
- D3 sequence 707 was interrupted and the next logged evaluation was new
  sequence 718 with `clock=bootstrap-candidate`.
- Earlier interruptions similarly moved from sequence 80 to 348 and sequence
  79 to 707; those later sequences could acquire their own new publication
  clock only after rejoining.

The dynamic run did not reproduce a full Overtake phase chain. It did exercise
the publication-boundary defect directly, which is the acceptance scope of this
Slice.

## Residual failures kept separate

The candidate run still entered prolonged Stop/Recovery states through Follow
hard-gap, terminal-contingency, steering-reachability and continuation proof
failures. Those states are not evidence that the interrupted clock should be
retained. They require a separate frozen snapshot and architecture comparison.

No parameter or safety-margin tuning is authorized by this result.

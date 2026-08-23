# Audit

## Baseline

- branch: `develop_july`
- parent: `eabe317`
- live evidence: `output/20260824-005436`
- user-owned dirty file excluded: `aichallenge/result-summary.json`

## Hypotheses

| Hypothesis | Evidence for | Refutation condition | Confidence |
|---|---|---|---:|
| Track/Cruise context uses the wrong residual policy | It alone is default-constructed; exact rows exceed local tolerance after `solved` | Row normalization causes repeated solve failure rather than stable canonical authority | rejected |
| Execution-primal boundary is too strict | Rejection is only a few millimetres/s or m/s2 | Values exceed the same solve's exact recorded row tolerance | rejected |
| OSQP settings/config are the root cause | Iterations often reach their configured limit | Same settings work for row-normalized Follow/Overtake and changing settings cannot repair ownership mismatch | low |
| Later pose displacement caused the stop | Vehicle eventually appears away from the line | Earliest Domain 2 rejection is before race session start | rejected |

## Authority invariant

The experiment changed only how the existing Track/Cruise canonical QP was
conditioned and certified. It has been removed after falsification; the same
canonical selector and final publisher remain authoritative.

## Dynamic evidence

- Experimental run: `output/20260824-011002`
- Domain 1: 33 certified / 32 solve failures.
- Domain 2: 10 certified / 9 solve failures.
- First startup failures also showed retained
  `window rejected: invalid-progress-evolution`.
- Later Cruise failures showed retained
  `dynamic-obstacle-present` despite no relevant overtake target.
- No tuning, clamp, fallback, timeout, or authority bypass was added.

## Root-cause boundary

The downstream execution-primal check is a detector. Row normalization proves
that the QP/warm-start path does not reliably converge within its own physical
input-row tolerances. The next Slice must audit row construction, initial
state/input equality, warm-start transport, and input-bound scaling before any
retained-world broadening.

# Evidence

## Candidate

- output-aligned SHA-256:
  `59db87f295b4572a956c49767a3276fe345010c3e2cda2ffa66f06c074458c11`
- frozen base SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- model output support: plus/minus 0.12 rad;
- train/early-stop split and every other optimizer setting: identical to
  DAgger v3.

The persisted audit is
`output/20260901-e2e-authority-aligned-v4.json`.

## Comparison

All figures below are commands available after the existing runtime bound.

| Scope | Metric | DAgger v3, 1.2 then clip | output-aligned 0.12 |
|---|---|---:|---:|
| aggregate | material sign | 0.936 | 0.934 |
| aggregate | material MAE improvement | 0.270 | 0.274 |
| aggregate | attainable utilization | 0.537 | 0.545 |
| held-out failed prefix | material sign | 0.959 | 0.950 |
| held-out failed prefix | material MAE improvement | 0.286 | 0.271 |
| held-out failed prefix | attainable utilization | 0.632 | 0.597 |
| failed last 200 | material sign | 0.939 | 0.939 |
| failed last 200 | material MAE improvement | 0.332 | 0.303 |
| failed last 200 | attainable utilization | 0.751 | 0.684 |
| peer | material MAE improvement | 0.333 | 0.299 |
| independent normal | residual MAE rad | 0.00670 | 0.00430 |

The aligned candidate never requires an external clip and reduces normal-state
leakage.  However, its small aggregate gain does not offset regressions on the
held-out failure tail and peer sequence.  It also remains below the existing
0.30 aggregate material-improvement gate.

## Decision

Reject output-aligned v4 and keep production frozen.  The training/runtime
support mismatch was real and is now visible, but constraining the model head
to 0.12 rad is not the root fix.  The held-out failure requires coherent
corrections larger than the current authority envelope during a short critical
interval, while ordinary states require near-zero leakage.

The next audit must inspect the closed-loop time series, not tune a global
bound.  It must distinguish:

1. correct model direction whose magnitude was truncated by authority;
2. wrong model direction that pushed the kart toward the wall;
3. correction arriving after the required geometry transition;
4. an otherwise correct correction overridden by longitudinal or simulator
   dynamics.

Only after this classification may a confidence/geometry-gated authority or a
different temporal policy be proposed.  No runtime file changed.

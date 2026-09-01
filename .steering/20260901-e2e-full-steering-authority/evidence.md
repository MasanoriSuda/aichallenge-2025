# Evidence

## Artifact and root-cause closure

- candidate: `spatial-production-wheel-base-seed2030-v11-full/20260901_213524/candidate.npy`
- SHA256: `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- embedded base SHA256: `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- model and runtime correction range: `+/-1.2 rad`

Frozen replay of the v10 failure shows the successor produces a mean
`+0.75353 rad` correction where the admitted teacher requires `+0.88240 rad`.
The rejected bounded model produced only `+0.11986 rad`.  This closes the
representation defect without a scenario-specific trigger or threshold.

## Offline acceptance

`output/20260901-e2e-wheel-base-seed2030-v11-full-audit.json` passed every
strict Gate:

- aggregate material MAE improvement: `35.99%`;
- runtime-bounded material sign accuracy: `94.95%`;
- peer material MAE improvement/sign accuracy: `68.53%` / `100%`;
- held-out focus material MAE improvement/sign accuracy: `37.59%` / `95.68%`;
- held-out tail material MAE improvement: `96.01%`;
- independent-normal MAE/P95: `0.009699 / 0.06335 rad`;
- maximum predicted correction: `0.94858 rad`, with no runtime-bound clipping.

## Closed-loop acceptance

| Gate | Laps [s] | Penalty / stall | Runtime evidence |
|---|---|---|---|
| shadow single | 98.930 / 87.751 / 87.856 | 0 / 0 | 7149/7149 admitted, error/stale 0 |
| authority single | 101.644 / 89.735 / 89.855 | 0 / 0 | 6645/6645 applied, clipping 0 |
| NPC seed 2026 | 103.718 / 89.515 / 100.240 | 0 / 0 | first place, 8556/8556 applied, clipping 0 |
| NPC seed 2027 | 103.418 / 90.010 / 100.225 | 0 / 0 | first place, 8859/8859 applied, clipping 0 |

Coverage was 100% and inference error/stale count was zero in every authority
run.  Maximum observed correction was `0.86176 rad`, confirming that the old
`0.12 rad` representation could not have executed the accepted behavior.

## Decision

Candidate v11 passes the defined offline, shadow, single and two-seed NPC
acceptance matrix and is eligible for packaging.  Candidate v10 and wheel-v4
remain rejected rather than being retained as fallback production owners.

The authority single run is slower than the corresponding shadow baseline.
This is a later performance-learning problem, not grounds to reintroduce the
structurally infeasible bounded representation.

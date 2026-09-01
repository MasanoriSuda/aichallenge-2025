# Evidence

## Replay identity

- Frozen production run: `output/20260902-e2e-final-packaged`
- Failed worlds: d1, d2
- Clean controls: d3, d4
- Known `slow-clearance` deadlock control:
  `output/20260902-e2e-final-recurrent-024/d1`
- Runtime reconstruction parity:
  - d1/d2 acceleration MAE below `6e-10 m/s2`
  - d3/d4 acceleration MAE below `4e-5 m/s2`

The fixed-policy replay therefore reproduces the command owner closely enough
for an authority comparison.  It remains an open-loop counterfactual and does
not establish the changed vehicle trajectory.

## Rejected global envelope

Applying the stopping envelope at every frontal distance was rejected.  With
effective deceleration assumptions of 1--3 m/s2 it intervened during about
10--75% of clean d3/d4 samples.  The 180-degree LiDAR sees course walls in
curves, so extending longitudinal obstacle authority beyond its existing
3.0 m exposure boundary would mix wall geometry with a new obstacle class.

## Selected bounded experiment

The slow-zone candidate keeps the existing boundaries:

- above 3.0 m: preserve the pace request;
- 1.5--3.0 m: limit acceleration by a continuous safe-speed envelope;
- at or below 1.5 m: preserve the existing `-1.0 m/s2` hard brake.

It does not increase intervention exposure relative to the fixed policy:

| world | fixed intervention | bounded intervention | longest interval |
|---|---:|---:|---:|
| d1 | 96.05% | 91.66% | 120.05 s |
| d2 | 96.37% | 91.79% | 120.05 s |
| d3 clean | 0.23% | 0.23% | 0.33 s |
| d4 clean | 0.00% | 0.00% | 0.00 s |

The long d1/d2 intervals begin after physical failure and cannot be interpreted
as prevention.  In the independent recurrent d1 deadlock, however, the frozen
policy commanded exactly `0.0 m/s2` for every one of 178 post-failure samples
at median front clearance 1.64 m.  The bounded candidate instead requested
`0.30--0.34 m/s2` for every sample.  This directly removes the zero-speed fixed
point without a creep state, timeout or retained escape command.

## Decision

- Reject the all-distance envelope.
- Admit only the 3.0 m-gated, `1.0 m/s2` effective-deceleration formulation to
  a separate runtime A/B Slice.
- Do not claim that it prevents the original d1/d2 wall contacts; lateral
  interaction data remain the upstream limitation.

## Verification

```text
python3 -m pytest -q test
217 passed in 1.72s
```

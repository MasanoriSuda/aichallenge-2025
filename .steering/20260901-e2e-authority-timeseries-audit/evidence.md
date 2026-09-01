# Evidence

## Frozen inputs

- Failed authority run: `output/20260901-180313`
- Failure validation sequence: final 200 samples of the recurrent sequence
  derived from that run, ending one second before the first confirmed LiDAR
  breach
- Production authority bound used by the failed run: `0.12 rad`
- v2: `spatial-production-normal-v2/20260901_171913/candidate.npy`
- v3: `spatial-production-normal-v3-dagger/20260901_183014/candidate.npy`
- v4: `spatial-production-normal-v4-authority-aligned/20260901_183736/candidate.npy`

The v3 training split contains candidate3, the original training set and the
successful authority run.  The failed authority run remains validation-only,
so the result below is not a replay leak.

## Causal target structure

The final 200 samples span about `9.96 s` and contain one sustained correction
reversal rather than high-frequency chatter:

| segment | samples | mean teacher residual |
|---|---:|---:|
| left | 98 | -0.16085 rad |
| neutral | 3 | approximately zero |
| right | 99 | +0.32001 rad |

The wall penalty starts at race time `282.413 s`.  The recorded controller
status first shows a strong left request, then an ambiguous transition, then a
right request while the vehicle is already committed near the wall.  The
result contains one wall event lasting approximately `137.35 s` and no crash
event.

## Candidate comparison

All delays require five consecutive predictions with the target sign.

| candidate | right-transition delay | tail sign accuracy | bounded residual gain at 0.12 rad | clipped fraction |
|---|---:|---:|---:|---:|
| v2 | 0.450 s | 0.898 | 0.322 | 0.715 |
| v3 DAgger | 0.000 s | 0.939 | 0.332 | 0.725 |
| v4 globally bounded | 0.450 s | 0.939 | 0.303 | 0.000 |

For v3, diagnostic bound sweeps produce residual gains of `0.332`, `0.427`,
`0.454` and `0.426` at bounds `0.12`, `0.18`, `0.24` and `0.30 rad`.
The aggregate improvement alone does not authorize a larger bound: v2 is
wrong-signed for about `0.45 s` at the reversal, so a larger global bound would
also amplify the wrong request during that interval.

## Classification

The evidence rejects two hypotheses:

1. The failure tail is not caused primarily by target chatter.
2. Globally shrinking model support to the runtime bound does not fix the
   transition and loses useful correction amplitude.

The strongest supported cause is a distribution/timing weakness in v2.  The
model remains left-biased after the teacher changes to the newly open side;
the runtime clamp is then saturated, but is not the origin of the wrong sign.
DAgger v3 removes that measured delay on a held-out failure sequence.

## Decision

- Keep candidate3 as production.
- Keep spatial authority disabled by default.
- Reject v4 and reject a blind global authority-bound increase.
- Advance v3 as the sole spatial candidate for the next shadow/acceptance
  stage.  It still needs an independent closed-loop run before authority can
  be granted.
- If v3 fails dynamically despite the zero-delay replay, the next architecture
  experiment must target temporal state or keyframe weighting rather than add
  a runtime debounce or another hand-authored steering patch.

Generated diagnostic reports:

- `output/20260901-e2e-authority-transition-v2.json`
- `output/20260901-e2e-authority-transition-v3.json`
- `output/20260901-e2e-authority-transition-v4.json`

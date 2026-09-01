# Evidence

## Frozen production baseline

- production candidate: `v11-full`
- packaged SHA-256:
  `f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c`
- authority range: `[-1.2, 1.2] rad`
- runtime settings and the packaged artifact were not changed by this audit.

## Closed-loop observation

The single-vehicle authority run was about 5.9 seconds slower over the measured
three laps than the corresponding shadow run.  Longitudinal acceleration was
the same and contained no negative command in either run.  Authority increased
steering total variation by about 11.6 percent and steering-rate p95 from about
2.19 to 2.34 rad/s.  The likely source of the pace loss is therefore normal
state steering intervention rather than braking.

On the clean shadow replay, v11 produced a correction of at least 0.02 rad on
about 10.2 percent of samples.  Most material corrections opposed the frozen
base steering command.

## Immutable normal-anchor corpus

`production_normal_anchor_v3_wheel_speed_current` was built from two admitted
base-authority train runs and one independent validation run.  The current v11
shadow run was used instead of the authority run so the zero-residual label is
owned by the frozen base controller rather than by the model being evaluated.

## Candidate comparison

| Metric | v11 production | v12 normal-heavy | v13 sequence-balanced |
|---|---:|---:|---:|
| Strict result | pass | pass | **fail** |
| Aggregate material MAE improvement | 35.99% | 34.44% | 28.75% |
| Material sign accuracy | 89.30% | 90.17% | 84.16% |
| Aggregate anchor false-material rate | 5.12% | 5.76% | 4.11% |
| Independent-normal MAE | 0.009699 rad | 0.007926 rad | 0.005304 rad |
| Held-out focus improvement | 37.59% | 35.80% | 27.10% |
| Held-out focus-tail improvement | **96.01%** | 12.55% | 13.28% |
| Peer material improvement | 68.53% | 46.56% | 51.08% |

The v12 clean replay reduced mean absolute correction from about 0.01251 to
0.00782 rad and p95 from about 0.08183 to 0.04775 rad.  However, on the frozen
v10 wall-stall failure suffix its mean correction fell from v11's +0.75353 rad
to +0.53948 rad while the teacher requested +0.88240 rad.  This weakens the
critical avoidance response by about 28.4 percent.

v13 tested whether an intermediate sequence balance could retain the critical
response while reducing clean intervention.  It failed the existing aggregate
material-improvement Gates and also retained the focus-tail regression.  It
was therefore not replayed dynamically.

## Root-cause classification

Increasing normal-anchor weight monotonically improves clean-state leakage but
simultaneously weakens the large correction required in the frozen failure
state.  The bounded v13 follow-up rules out v12 being only an unlucky single
data ratio.  Further sampling-weight tuning would be candidate churn rather
than a root-cause fix.

The remaining problem is classified as a representation/separability limit in
the current static single-frame soft-mixture output.  The next experiment must
make neutral/no-intervention an explicit learned mode (or add temporal context)
rather than introduce a runtime deadband, trigger, or hand-written threshold.

## Decision

- Reject v12 and v13.
- Keep v11 as the production artifact.
- Do not run shadow or authority closed-loop tests for rejected candidates.
- Do not change runtime authority, clearance, timeout, or fallback settings.
- Open a separate architecture slice before training another candidate.

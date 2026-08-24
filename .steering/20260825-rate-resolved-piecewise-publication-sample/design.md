# Design

## Time ownership

Prediction stage durations and the 40 Hz publication interval remain immutable
inputs. The sampler does not enlarge stage zero to fit the publisher. It walks
the certified horizon:

```text
remaining = publication_interval
steering = semantic_current_steering

for each stage i:
  step = min(remaining, stage_duration[i])
  steering += certified_rate[i] * step
  validate actual steering
  if remaining <= stage_duration[i]: publish steering
  remaining -= stage_duration[i]
```

Constant rate makes steering monotonic within each stage. Validating every
crossed endpoint and the final partial endpoint therefore validates the whole
integrated publication prefix.

## Certificate boundary

- The whole-QP physical row certificate owns solved rates, predicted states
  and dynamics.
- The preceding first-rate Slice guarantees the first full stage is reachable
  from semantic current steering despite numerical state-zero residual.
- This sampler owns only conversion of the certified piecewise rate sequence
  to the real publication time and the actual integrated steering bound.
- It does not re-check rates with a second tolerance or use reconstructed
  solver state as a new integration origin.

## Failure behavior

- malformed/nonfinite stage data fails closed with its existing typed reason;
- publication beyond the entire certified horizon uses a new typed
  `publication-after-horizon-end` reason;
- actual integrated boundary violation remains
  `sampled-steering-limit-violation`;
- no sample is clamped.

## Authority

The result remains observation-only. Telemetry must continue to state
`authority=shadow, selected=0`.

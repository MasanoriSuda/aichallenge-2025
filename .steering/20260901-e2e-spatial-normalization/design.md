# Design

```text
teacher train scans + normal train scans
  -> frozen candidate3 conv5 map
  -> per-feature train mean and standard deviation
  -> immutable candidate buffers
  -> fixed standardization
  -> existing signed continuous correction head
```

This differs from LayerNorm: fixed statistics normalize each spatial feature
using train-only population evidence, while preserving between-sample
activation scale.  The same buffers are loaded during evaluation and are part
of strict checkpoint identity.

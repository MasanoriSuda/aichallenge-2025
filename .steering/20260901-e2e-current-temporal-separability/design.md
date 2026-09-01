# Design

The prior temporal probe predates the qualified full-authority corpus and the
current production-normal contract.  Re-run the existing causal representation
probe on the exact data that exposed the v11/v12 trade-off.

The controlled comparison is:

- `static_conv5`: current spatial map plus synchronized wheel speed;
- `temporal_conv5`: the same current spatial map plus causal lagged spatial
  differences and synchronized wheel speed;
- corresponding base-steering-conditioned variants as an additional check.

Every history resets at run boundaries.  Three deterministic projections and
optimizations are required.  This remains a classifier diagnostic, not a
runtime checkpoint.

If temporal evidence is consistently better, the next Slice may build a
frozen-base temporal spatial adapter.  If it is not, another GRU/history model
would repeat a disproven direction and the data/teacher observability contract
must be audited instead.

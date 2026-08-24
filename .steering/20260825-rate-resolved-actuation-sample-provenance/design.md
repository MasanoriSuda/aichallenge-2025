# Design

## Causal boundary

```text
semantic request -> QP assembly -> certified solve -> actuation sampling
                                               PASS            FAIL
```

The first three stages succeeded in every consumed runtime result. This Slice
therefore changes only the observability of the final sampling boundary.

## Typed result

`evaluate_actuation_sample()` returns a reason plus the calculated terminal and
publication-time steering values. `sample_actuation()` remains a thin wrapper
returning the accepted optional sample, so existing callers retain identical
behavior.

Reasons distinguish invalid/non-finite inputs from these executable boundaries:

- publication time beyond the first optimized stage;
- observed steering outside the physical box;
- optimized steering rate outside the physical box;
- first-stage terminal steering outside the physical box;
- 40 Hz sampled steering outside the physical box.

The shadow aggregates reason counts and records the last exact values. It never
uses the diagnosis to repair, clamp, select or publish a solution.

## Decision rule

Dynamic evidence selects the next root-cause Slice:

- time rejection -> repair stage/publisher time-base ownership;
- input or terminal boundary rejection within certified row residual -> align
  numerical boundary normalization with the solver certificate;
- material physical violation -> repair QP row construction;
- invalid/non-finite input -> repair semantic adapter provenance.

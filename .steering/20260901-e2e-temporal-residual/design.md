# Design

## Root cause under test

At the four-peer failure boundary, nearby single-frame LiDAR observations
required opposite corrections.  More epochs, equal-run sampling and both-side
training drove the stateless correction toward zero or leaked into anchors.
The next smallest causal hypothesis is missing motion context, not a clearance
or braking threshold.

## Feature contract

For normalized scans `s[t]` and `s[t-1]`, the temporal model input is:

1. channel 0: `s[t]`;
2. channel 1: `s[t] - s[t-1]`.

At the first sample of each sequence/runtime, `s[t-1] = s[t]`, so the delta
channel is exactly zero.  History never crosses a dataset sequence boundary.

## Architecture boundary

The existing two-head bounded residual architecture is retained except for the
first convolution accepting two channels.  Runtime selection is explicit:

- `stateless`: one current-scan channel;
- `scan_delta`: current and scan-delta channels.

Checkpoint key and shape validation stays strict.  Empty residual checkpoint
still means no residual model is constructed.

## Admission

Offline evaluation must include:

- both new d1/d2 final-10-second corrective subsets;
- historical d3 anchor tail;
- full held-out residual validation;
- independent normal-run leakage.

Only a candidate satisfying those gates may run single vehicle, NPC and then
four peer.  A failure closes the Slice without changing production.

## Outcome

The scan-delta model learned both opposite-action failure tails but leaked
steering into normal and anchor samples by nearly an order of magnitude over
the admission limit.  The offline gate therefore rejected it before runtime.
See `evidence.md`; the production policy remains frozen.

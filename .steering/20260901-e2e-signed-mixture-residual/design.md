# Design

## Root cause under test

The binary residual asks one regression head to represent both pass sides, then
uses a binary material gate.  Similar observations with opposite teacher signs
can therefore average toward zero, while aggressive fitting opens the same gate
on ordinary states and leaks a large arbitrary sign.

## Model

The encoder and `scan_delta` input remain frozen for the comparison.  The head
becomes:

- `direction_head`: negative / anchor / positive logits;
- `magnitude_head`: non-negative magnitude for negative and positive direction;
- composed residual: `p(positive)*magnitude_positive -
  p(negative)*magnitude_negative`.

Equal initial logits and magnitudes cancel exactly.  Ambiguous side selection
also tends toward zero rather than an arbitrary signed correction.

Direction class weights are computed from the mean per-run class mass so they
match the existing run-balanced sampler.  Magnitude loss applies only to
material samples; anchor leakage remains explicitly penalized.

## Boundary

Only Torch training/evaluation is implemented first.  NumPy runtime, launch
wiring and closed-loop gates are prohibited until offline admission succeeds.

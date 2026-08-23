# Design

## Status

Rejected after dynamic acceptance.  This document records the tested
formulation and why it must not be reintroduced unchanged.

## Causal chain

```text
physical progress rate != virtual progress speed
  -> e_lag accumulates while theta advances independently
  -> reconstructed pose is based on theta and shifted by e_lag
  -> old QP still uses wall bounds for nominal stage index
  -> optimizer and physical certificate inspect different course positions
  -> solved candidate is rejected, or authority arrives after wall contact
```

Representative evidence from `output/20260823-095004`:

- decision 1581: nominal/solved progress `125.737/124.802 m`, lag `0.344 m`;
- decision 2442: nominal/solved progress `288.503/287.507 m`, lag `-0.175 m`;
- both were accepted numerically and rejected by the swept physical wall
  certificate near stage zero.

The earlier `20260822-track-cruise-wall-bound-contract` Slice intentionally
fixed only the certificate's course-frame lookup and left the QP lateral boxes
as fixed centre-path constraints.  This Slice tested an attempt to close that
remaining upstream contract mismatch; the attempt was rejected for solver and
real-time regressions recorded in `validation.md`.

## Constraint model

For each predicted state stage, interpolate the lower and upper lateral
corridor as functions of unwrapped course progress.  Around reference progress
`s_ref`:

```text
lower(s_phys) ~= lower_ref + lower_slope * (s_phys - s_ref)
upper(s_phys) ~= upper_ref + upper_slope * (s_phys - s_ref)
s_phys         = progress_origin + theta + lag
```

This gives two affine rows:

```text
e_y - lower_slope * lag - lower_slope * theta
  >= lower_ref - lower_slope * (s_ref - progress_origin)

e_y - upper_slope * lag - upper_slope * theta
  <= upper_ref - upper_slope * (s_ref - progress_origin)
```

The predicted `e_y` box rows become broad numerical bounds rather than a
second fixed-stage corridor.  Stage zero remains fixed by the measured-state
equality and is checked by the existing current-pose wall certificate.

## Boundary gradient contract

- Progress samples must be finite and strictly increasing.
- Bounds must be finite and ordered at every sample.
- Endpoint stages use one-sided slopes; interior stages use the secant between
  adjacent samples.
- The generated coefficient and RHS values must be finite.
- A malformed corridor rejects extended problem construction; it is not
  silently replaced with the old fixed-stage contract.

## Remaining approximation

`theta + lag` is the first-order physical progress coordinate of this model.
On a curved course, translating by lag along the `theta` tangent is not exactly
the Frenet pose at `theta + lag`.  The exact oriented-footprint and swept-path
certificate remains the final safety oracle for that nonlinear remainder.  No
certificate is weakened in this Slice.

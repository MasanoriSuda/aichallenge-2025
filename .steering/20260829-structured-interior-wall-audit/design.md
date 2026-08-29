# Design: structured interior-wall audit

## Root cause under test

The exact proof integrates the nonlinear seven-state model every 10 ms. The
live QP currently represents each transition interior using four affine
endpoint-interpolation rows. Dense nonlinear tangents close the Follow proof
but are too expensive and do not solve the frozen ShiftOut case. Adding one
failed sample at a time merely moves the violation.

## Structured arm

For each transition, select up to four unique physical-integration substeps
nearest one-fifth, two-fifths, three-fifths and four-fifths of the interval.
Build a partial-duration seven-state tangent for each selected substep and
constrain its lateral output against the same interpolated wall interval used
by exact proof.

Before assembly, remove the old affine swept rows. The selected nonlinear rows
replace them, so the comparison does not hide a row-budget increase.

## Classification

- structured solve + exact proof accepted: sparse representation defect;
- structured solve + proof rejected: four-point topology is insufficient;
- structured numerical rejection: conditioning/single-SQP remains;
- dense succeeds while structured fails: dense oracle is not reducible to the
  tested fixed four-point representation.

No outcome changes production in this Slice.

# Results: circular-seam terminal Stop audit

## Baseline observation

Run `output/20260831-183224` reproduced an ordinary Cruise interruption at
decision 4577.  The preceding artifact 3083 was still physically joined and
had a certified Stop successor.  A fresh current-world artifact could not
construct its terminal Stop geometry while the reference crossed the circular
course seam.  The already-certified Stop was therefore correctly selected;
the downstream Stop retention is a symptom, not the initiating defect.

## Audit instrumentation

The fail-closed geometry rejection now reports:

- nominal state count;
- solver input count;
- declared horizon;
- wall progress/lower/upper counts;
- wall refinement activation;
- original wall profile construction diagnostic.

No authority, candidate, Stop, solver, clearance, timeout, lease, grace or
fallback behavior changed.

## Static verification

- `make autoware-build`: passed, 25 packages built.
- complete `multi_purpose_mpc_ros` CTest suite: 59/59 passed.
- source contract requires all geometry-owner fields to remain observable.

## Pending dynamic classification

The next bounded dev2 run must reproduce the seam and use the new field counts
to decide whether the root defect is horizon truncation, missing wall profile,
or a circular-stage provenance mismatch.

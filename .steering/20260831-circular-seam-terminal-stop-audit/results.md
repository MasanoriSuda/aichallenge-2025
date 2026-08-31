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

## Bounded dynamic result

Run `output/20260831-184430` crossed the circular seam and emitted the normal
`circular seam normalized` records at waypoints 330, 338 and 347, but did not
emit `terminal Stop course geometry unavailable`.  The baseline defect was
therefore not reproduced and no shape/provenance repair is justified from this
run.

The run later lost normal authority for a different, independently traceable
reason at decision 1116: a low-speed Stop successor was rejected as
`exact-trajectory-rejected/exact:invalid-path-distance`.  That failure is
owned by the separate `20260831-stop-stationary-trajectory-contract` Slice so
the two snapshots are not conflated.

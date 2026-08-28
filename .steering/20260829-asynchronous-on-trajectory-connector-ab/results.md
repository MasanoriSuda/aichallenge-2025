# Results

## Dynamic run

- Run: `output/20260829-022011`
- Mode: `make dev2`
- Production authority and tuning parameters were unchanged.
- Build: 25 packages passed.
- Test: 52/52 test targets, 2078 assertions, no failures.

## A/B counts

| Vehicle | Comparisons | A time-aligned rejected / B origin accepted | B initial-lateral-bound rejected |
|---|---:|---:|---:|
| d1 | 930 | 253 | 653 |
| d2 | 161 | 23 | 130 |

Every B-arm result reported `origin_authority=0` and `selected=0`.

Representative startup evidence:

```text
sequence=1
time_aligned=steering-unreachable/elapsed=0.065/expected=-0.0464
origin=accepted/elapsed=0.000/expected=0.0000
origin_authority=0, authority=observation-only, selected=0
```

The worker produced a complete 20-stage physical proof at the artifact origin.
After 65 ms of asynchronous computation, the current vehicle had not executed
the candidate's steering prefix, so selecting the elapsed suffix required an
unreachable `-0.0464 rad` command.

The largest remaining B-arm class was
`continuation-rejected/initial-lateral-bound-rejected`.  Resetting only the
clock repaired steering reachability but could not repair the fact that the
candidate's state-zero geometry belonged to a prior predicted execution pose.
This is the same connector divergence expressed in the lateral state.

## Classification

This is a **scheduling/on-trajectory connector defect**:

- the optimization and exact certificate can succeed;
- production current-world proof correctly rejects a trajectory whose skipped
  prefix was never published;
- neither elapsed-time suffix selection nor replaying cursor zero supplies a
  causal physical connection.

It is not evidence for changing wall clearance, steering rate, solver
tolerance, Mission lease, or timeout.

## Decision

Do not publish either diagnostic arm.  The next Slice must make the new solve
share an explicit prefix/switch state with the last actually published
certified artifact, or add an AS-RTI-style latest-state feedback correction.
The temporary A/B evaluator should be removed now that the classification is
frozen, because it duplicates current-world proof work in the 40 Hz callback.

The required trace is:

```text
MPCC_ASYNC_CONNECTOR_AB: ...
time_aligned=...
origin=...
origin_accepted=...
origin_authority=0
authority=observation-only, selected=0
```

Acceptance was met: `origin_authority=0` in every comparison.  The production
decision remained the time-aligned or already-published certified artifact.

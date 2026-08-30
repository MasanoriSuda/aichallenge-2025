# Requirements: primary retained proof tail audit

## Objective

Classify the remaining 21--29 ms single-plan retained proof tail after
diagnostic filesystem I/O was removed from the control callback.

## Frozen evidence

- Baseline: `ac121794`.
- Run: `output/20260831-011806`.
- d2 decision 1631: primary retained 21.372 ms.
- d1 decision 3330: primary retained 29.325 ms.
- Both cycles evaluated one plan and spent only 0.002 ms in failure snapshot
  capture.

## Constraints

- Observation only.
- No authority, solver, clearance, Mission, Stop or scheduling change.
- Reuse the existing typed runtime breakdown.  The first run showed that its
  combined `continuation_proof` region was still ambiguous, so extend that
  same typed structure with non-overlapping delay-wall, dynamic and
  continuation-wall subregions; do not add a second timing mechanism.

## Definition of done

- Slow production-join logs show the aggregate retained proof regions.
- Build and all tests pass.
- A bounded dev2 run captures at least one slow primary proof or establishes
  a measured upper bound.
- The next Slice is selected from the measured dominant region.

## Result

- `output/20260831-012707`, d2 decision 1308:
  `continuation_proof=23.350 ms` dominated a `27.612 ms` production join.
- The typed subregion instrumentation was then added without changing
  authority or proof behavior.
- `output/20260831-013511`, d2 decision 3426:
  - continuation wall proof: `12.917 ms`
  - terminal Stop wall proof: `5.152 ms`
  - continuation dynamic proof: `0.227 ms`
  - delay wall proof: `0.201 ms`
- The remaining retained-proof tail is therefore a synchronous exact wall
  proof cost, not dynamic-obstacle prediction, candidate construction or
  diagnostic I/O.

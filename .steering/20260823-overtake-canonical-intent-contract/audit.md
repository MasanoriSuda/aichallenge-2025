# Root-cause audit

## Hypothesis

The fresh Overtake artifact chain is blocked by an incomplete shared intent contract, not by solver
quality or physical-wall infeasibility.

## Supporting evidence

- 352/352 extracted exact trajectories passed swept physical wall certification.
- 0/352 could construct a canonical plan.
- The first canonical validator checks `canonical_normal_intent_supported()`.
- The shadow eligibility contract admits only ShiftOut/Pass/Return.
- The shared support function admits only Track/Cruise/Follow.

## Falsification

After extending the contract, replay must either build complete canonical shadow selections or
report a different exact rejection. Continued `UnsupportedIntent` means the root cause or runtime
intent identity is wrong.

## Secondary defect prevented

The current completeness predicate requires a target only for Follow. Extending intent support
without also requiring Overtake target provenance would allow targetless canonical Overtake plans.
The target rule is therefore part of the same root-cause contract, not a later fallback patch.

## Dynamic conclusion

The same bounded replay produced 404 shadow evaluations. 390 were eligible Overtake execution
cycles and 353 satisfied the exact lateral-row contract. All 353 then completed primal
normalization, actuation/trajectory extraction, swept physical certification, canonical
plan/cursor/candidate/authority/command construction and world prediction.

`unsupported-intent` fell from the previous all-artifact rejection to zero. The exact first
actuation difference remained zero. This confirms the hypothesis.

The next observed boundary is independent: 37 eligible results fail the stage-zero lateral-row
contract, typically with 0.057--0.097 m violation against about 0.0164 m solver tolerance. That
failure must be audited from x0/bound construction before any authority promotion. It is not a
reason to weaken this intent contract or tune the tolerance.

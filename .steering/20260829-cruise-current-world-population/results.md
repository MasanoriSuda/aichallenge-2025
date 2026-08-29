# Results: Cruise/Follow current-world avoidance population

## Observed failure

Frozen Cruise sequence 601 contained an active dynamic-obstacle contract but
production evaluated only the captured neutral automatic branch.  That branch
reached the solver with mutually inconsistent dynamic-lateral and
progress-dependent wall rows and was rejected.  This was visible as solver
failure, but the solver was not the producer of the invalid topology.

The pre-fix authority graph was asymmetric:

```text
Follow + dynamic obstacle -> positive/negative current-world population
Cruise + dynamic obstacle -> neutral automatic branch only
```

The architecture comparison initially had the same detection gap: Cruise was
rejected as `unsupported-intent` before either alternative was built.

## Root cause

The first violated invariant was candidate completeness.  A normal dynamic
obstacle candidate must solve its complete homotopy, dynamics,
progress-dependent wall envelope and obstacle disjunction together.  Neutral
Cruise instead derived one behind-to-side obstacle schedule from scalar state
boxes and treated it as the sole topology.

The same immutable sequence falsifies physical infeasibility:

| Candidate | Result | Terminal progress | Terminal velocity | Lateral reserve |
|---|---|---:|---:|---:|
| captured neutral | solver rejected | N/A | N/A | N/A |
| positive side | all exact proofs accepted | 9.41925 m | 5.90443 m/s | 1.21804 m |
| negative side | all exact proofs accepted | 9.47662 m | 5.90443 m/s | 1.45082 m |

## Implemented producer replacement

- Generalized the existing Follow current-world population to Cruise/Follow.
- Dynamic-obstacle Cruise and Follow now rebuild both physical side candidates
  from the same immutable world before the ordinary direct solve can run.
- Retained one numerical context per physical side and one bounded homotopy
  continuity owner inside the existing latest-only worker.
- Kept `intent=Cruise|Follow` and `execution_side_sign=0`; candidate side is
  provenance for rows and refinement, not Overtake Mission authority.
- Reused the unchanged seven-state solve, exact nonlinear/wall/dynamic and
  terminal proof chain, certified Store and normal publisher.
- Deleted the Follow-only producer/API, context and owner naming.  No worker,
  publisher, fallback, parameter or normal authority was added.

The neutral automatic low-level implementation remains for compatibility and
offline callers, but the source contract proves it is no longer the production
path for an active Cruise/Follow dynamic-obstacle problem.

## Verification

- failing replay: frozen Cruise sequence 601 captured neutral arm rejected;
- repaired replay: both rebuilt side arms accepted by unchanged exact proofs;
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q .../test_single_authority_source_contract.py`:
  75 passed;
- `make autoware-build`: 25 packages passed;
- full package CTest: 54/54 targets passed;
- `git diff --check`: passed.

## Deletion and configuration accounting

- new production authorities: 0;
- new workers or publishers: 0;
- new flags, leases, timeouts, retries or fallbacks: 0;
- changed solver/clearance/weight configuration: 0;
- removed production path: Cruise dynamic obstacle -> single neutral automatic
  candidate;
- removed API/ownership duplication: Follow-only normal avoidance producer and
  owner names.

## Remaining concerns

This Slice repairs the classified Cruise candidate-generation defect.  It does
not repair frozen ShiftOut sequence 1266, which the independent nonlinear
oracle classified separately as a single-SQP/convexification limitation.
Dynamic race acceptance must additionally confirm that current-world Cruise
obstacles emit `normal-avoidance-positive|negative` certified artifacts without
new callback overruns or Emergency tails.  Those observations are a validation
gate, not justification for restoring the deleted neutral production path.

Rollback commit: `00f5f97b`.

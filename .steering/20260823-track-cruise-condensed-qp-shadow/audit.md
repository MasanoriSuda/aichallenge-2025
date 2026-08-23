# Audit

## Decision

Rejected and removed. Exact affine condensing is not a viable replacement for
the current expanded Track/Cruise QP under the unchanged OSQP numerical
contract.

The shadow never owned a command, canonical plan, retained candidate, or
publisher. The expanded five-state Track/Cruise solver remained the only normal
production authority throughout the experiment.

## Static evidence

- Failure-first compilation demonstrated that no condensing/reconstruction API
  existed before this Slice.
- Pure tests proved objective/constraint equivalence for arbitrary inputs,
  exact dynamics reconstruction, malformed-layout rejection, and semantic
  condensed warm-start shifting.
- `make autoware-build`: 25 packages built successfully.
- Focused tests passed.
- Full `multi_purpose_mpc_ros` test run: 38/38 CTest entries and 1,635 test
  results passed with zero failures.

## Dynamic evidence

Run: `output/20260823-115811`, Domain 1, AWSIM `make dev`.

Across the 37 complete one-second telemetry windows:

| Metric | Condensed observer | Expanded production |
|---|---:|---:|
| eligible cycles | 1,523 | 1,523 |
| solver results | 1,523 | 1,523 |
| semantic execution primals accepted | 889 (58.37%) | 1,488 (97.70%) |
| maximum reconstructed dynamics residual | `1.55e-14` | not applicable |
| maximum expanded physical-unit violation | `0.01795` | production diagnostics retained |
| maximum normalized expanded violation | `16.44` | production diagnostics retained |

The observer had no solve failures or non-finite results. Its maximum solver
time was 3.383 ms and maximum total observer time was 4.209 ms. The control
callback recorded 2,107 cycles, zero overruns, and a maximum of 21.042 ms
against the 25 ms period.

The dominant observer rejection was a certified acceleration or curvature box
violation, commonly at stage 0 but also at later stages. Typical absolute
violations were 0.003--0.018, while reconstructed dynamics equality residuals
remained at floating-point roundoff.

## Causal conclusion

The experiment separated two effects that the expanded formulation previously
mixed:

1. Exact affine elimination does remove approximate dynamics equality error.
2. It does not improve the executable input-bound contract. With state
   elimination, the objective and state bounds become dense functions of every
   input stage. Under the unchanged first-order solver termination contract,
   input box residuals became materially worse even though the dynamics were
   exact.

Therefore approximate dynamics equality was a real numerical defect in the raw
expanded solution, but it was not the dominant cause of current production
semantic rejects. The proposed structural replacement would trade that defect
for much larger actuator-bound violations and reduce execution availability by
about 39 percentage points.

## Why no rescue was attempted

No OSQP tolerance, iteration limit, scaling, polish, weight, clamp, projection,
retry, flag, or fallback was changed. Such a change would create a second
variable contract and invalidate the exact comparison. The Slice acceptance
rule required removal when semantic rejects were not reduced.

## Removed implementation

The following observation-only experiment code was removed after the dynamic
falsification:

- exact expanded-to-condensed QP builder and reconstruction API;
- condensed warm-start shifting;
- dedicated condensed solver context;
- observer solve and comparison telemetry;
- experiment-only tests and solver-tolerance telemetry exposure.

No production authority or runtime configuration was changed by the final
commit.

## Remaining root-cause direction

Do not continue with input condensing or attempt to rescue it through parameter
tuning. The next Slice must return to the canonical expanded formulation and
identify why its small explicit actuator/state boundary errors occasionally
cross the semantic certificate. A structurally valid next observation should
preserve the expanded sparsity and authority contract rather than transform the
entire QP coordinate/layout again.

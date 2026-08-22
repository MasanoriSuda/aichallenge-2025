# Validation

## Temporary full-margin A/B

Run: `output/20260822-172121`

The local and cloud YAML were temporarily changed from `safety_margin_scale: 0.0` to `1.0`, rebuilt,
and started with `make dev`. Track/Cruise remained `authority=shadow, selected=0`. The YAML values
were restored to `0.0` immediately after the run.

AWSIM reached `spawned -> grounded -> ready` and the vehicle ran from wp29 to wp273. It then made
wall contact and entered Stuck Recovery, so this run is intentionally not a lap-performance result.

Aggregated before/through the incident:

- shadow eligible / attempted: 1,953 / 1,953;
- solved: 1,931;
- physical checks: 1,923;
- full certificates: 1,902;
- physical rejects: four candidate contacts and 17 unsafe current-pose contacts;
- solve maximum: 22.232 ms;
- shadow maximum: 24.349 ms;
- callback maximum: 36.021 ms, 48 overruns.

The first logged candidate contact was wp270 stage 0, lateral 1.216 m, narrowed scalar bounds
`[-4.106, 1.499]` m and 0.283 m scalar reserve. The production pose contacted the wall shortly
afterwards. Both legacy and shadow then developed stage-0 constraint/solve failures because the
current state could not reach the abruptly narrowed next-state interval.

## Interpretation

The experiment supports the claim that the zero-margin centre-point contract contributes to the
normal-run candidate contacts, but falsifies a global config change as the fix:

1. A fixed scalar margin still did not prove the oriented body safe in the curved wp270 region.
2. Applying the margin to legacy production changed its feasible set and caused a wall incident.
3. Applying an already narrowed physical interval to the first predicted state can make the hard
   current-state equality dynamically unreachable.

The remaining upstream ambiguity is five-state progress geometry. The QP solves a free progress
state, but the certificate currently reconstructs every pose at fixed `ref_wp_id + stage`. If solved
progress differs from nominal stage progress, bounds and wall samples are evaluated in different
course frames. Progress provenance is therefore added before selecting the structural fix.

## Progress provenance run

Run: `output/20260822-173527`

The new diagnostic confirmed that the certificate was evaluating the solved state in a nominal
course frame ahead of the solved progress. Representative failures were:

- decision 6876: swept stage 3, wp260, reference/solved progress 255.114/257.016 m, delta -1.903 m;
- decision 6880: hard contact stage 0, wp258, reference/solved progress 254.037/255.035 m,
  delta -0.998 m, while the scalar QP reserve was 1.065 m.

The aggregated run contained 8,986 eligible/attempted/solved shadows, 8,903 certificates, 18
candidate hard contacts, 58 unsafe current-production poses and seven swept failures. This directly
supports the progress-frame hypothesis.

## Failure-first contract tests

Before implementation, `test_mpc_stage_geometry` was extended to require:

- interpolation at solved progress;
- shortest-angle heading interpolation across the wrap;
- rejection outside the provenance window;
- endpoint clamping only within the accepted solver tolerance.

The test failed to compile because the typed course-frame contract did not exist. After
implementation, the geometry and execution-contract tests passed.

## First structural run and tolerance finding

Run: `output/20260822-180609`

The initial structural implementation solved all 3,618 eligible cycles and certified 2,936. The
remaining 682 rejects were all `course-frame-unavailable`. OSQP had accepted millimetre-scale
equality residuals at the progress origin, while the first sampler used an unrelated `1e-9`
tolerance. This was a numerical contract mismatch, not a new wall failure.

The sampler now consumes the same accepted metre-domain solver tolerance. A deterministic test
proves that 9.999 m clamps to a 10.000 m origin when tolerance is 0.002 m, while 9.997 m is rejected.

## Corrected repeated shadow validation

Run: `output/20260822-181304`

The run continued through the previously failing wp258--261 zone and was stopped normally. Results:

- eligible / attempted / solved / physical checks: 4,794 / 4,794 / 4,794 / 4,794;
- full certificates: 4,782 (99.7497%);
- candidate discrete hard contacts: 0;
- course-frame unavailable: 0;
- current production pose hard contacts: 10;
- genuine current-to-first-stage swept failures: 2;
- all invalid/bound/heading/sample categories: 0;
- shadow selections: 0 (`authority=shadow`).

At decision 3767 the remaining candidate-side reject was a genuine swept collision at path index 1:
the first solved pose itself was clear, but the current production pose to that pose crossed the wall
near wp260. Decision 3769 then reported the production pose itself in hard contact. This is now a
separate reachability/stitch defect for the next slice rather than a false discrete candidate contact.

## Conclusion

The full-margin config A/B was useful but is not the architectural fix. The confirmed root cause was
the course-frame mismatch between solved progress and fixed nominal stage geometry. Using solved
progress provenance removed candidate discrete hard contacts from the measured run without weakening
clearance or changing production authority. The remaining work is the first-stage physical connection
and the contract for handing control from an unsafe legacy-created current pose.

## Final static validation

- `make autoware-build`: passed; 25 packages completed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 33/33 CTest targets passed.
- `colcon test-result --verbose`: 1,558 tests, zero errors, failures or skips.
- The pre-existing stale `build/joycon_contract_guard/package.xml` result warning remained benign.
- `git diff --check`: passed.
- `aichallenge/result-summary.json` remained an unrelated, unstaged user change.

## Excluded run

`output/20260822-171910` remained in AWSIM `spawned` state and never formed a valid race run. It is
excluded from all control conclusions.

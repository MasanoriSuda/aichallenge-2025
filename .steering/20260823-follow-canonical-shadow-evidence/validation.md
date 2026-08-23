# Dynamic validation

## Run

- command: `make dev2` with a temporary Compose overlay that changed only the simulator working directory
- controller logs: `output/20260823-164329/d1/autoware.log`, `output/20260823-164329/d2/autoware.log`
- isolated result: `output/20260823-follow-canonical-shadow/`
- user-owned `aichallenge/result-summary.json` SHA-256 before/after:
  `03e2f3935d95a550d0e1a3f2006dde08dae4a7a4c74121c430b2452daf4414e6`

The run was stopped after sufficient Follow encounters were collected. It was not a lap-performance
acceptance run. Domain 1 recorded three crash penalties under the unchanged production controller;
the Follow canonical result was always shadow-only and therefore cannot have commanded those events.

## Aggregated Follow shadow result

Domain 1 produced 97 one-second telemetry windows:

| Boundary | Count | Rate versus eligible |
|---|---:|---:|
| eligible | 3239 | 100.00% |
| typed contract accepted | 3110 | 96.02% |
| QP attempted | 3110 | 96.02% |
| QP solved and normalized | 2573 | 79.44% |
| actuation extracted | 2573 | 79.44% |
| effective physical gap accepted | 2573 | 79.44% |
| swept physical wall proof accepted | 2573 | 79.44% |
| canonical plan/cursor/candidate/authority/actuation/command | 2573 at every boundary | 79.44% |

The rate after a valid contract was 82.73% (`2573 / 3110`). Every solved and normalized primal passed
the physical and canonical boundaries; the first material loss was the QP solve boundary, not wall,
gap, canonical conversion, or authority selection.

Runtime over the aggregated windows:

- build: 0.047 ms average, 2.279 ms maximum
- solve: 5.686 ms average, 37.728 ms maximum
- physical/canonical certificate: 0.714 ms average, 10.610 ms maximum
- total shadow evaluation: 6.111 ms average, 39.587 ms maximum
- warm starts applied: 0
- solver-context resets: 41
- post-solve row rejects: 27 (velocity 4, other 23; transition logs identify the latter as input acceleration)

All Follow shadow event logs reported `authority=shadow, selected=0`. All successful chains reported
`canonical=1/1/1/1/1/1` and exact direct-primal/canonical-actuation agreement.

Domain 2 did not obtain a coherent front observation during this run, which is consistent with it being
the non-following vehicle in the collected scenario. It did not contribute eligible Follow samples.

## Root cause found

The zero warm-start count is a structural integration defect introduced when the physical Follow gap was
added as `N + 1` trailing constraint rows:

1. `build_extended_progress_problem()` correctly appends the physical constraint
   `progress + lag <= target_progress - hard_gap` for every state stage.
2. `solve_extended_progress_problem()` stores a successful primal and dual, then asks
   `persistent_osqp::shift_mpc_warm_start()` to shift them on the next cycle.
3. `shift_mpc_warm_start()` accepts only the original dual layout
   `dynamics + state/input boxes + N rate rows`.
4. A Follow dual now contains an additional `N + 1` rows, so the exact-size check returns `nullopt`.
5. The persistent solver is therefore explicitly cold-started on every numeric update.
6. Under the moving Follow bounds, this produces repeated `maximum iterations reached` failures and
   occasional one-cycle physical row residual rejects, followed by recovery on a later cold solve.

This explains the observed combination of `warm=0`, 537 failed solve attempts, and fully successful
physical/canonical processing whenever the QP did solve. Weakening the hard gap, wall margin, OSQP
tolerances, or canonical gates would address the wrong boundary.

The 129 contract rejects are primarily `initial-hard-gap-violation`. They are an intentional refusal to
invent a feasible trajectory when the measured starting gap is already below the hard constraint and are
not included in the solver-root-cause diagnosis.

## Next slice

Make the warm-start shift contract describe and shift the complete dual row layout, including typed
trailing stage-wise blocks. The Follow builder must declare its `N + 1` physical-gap block explicitly;
the solver must not infer or silently append unknown rows. Add unit tests for:

- the legacy base layout remaining unchanged;
- one `N + 1` state-stage trailing block shifting correctly;
- invalid or undeclared trailing rows being rejected;
- the Follow shadow using warm starts without changing production authority.

No solver setting or driving parameter should change in that slice.

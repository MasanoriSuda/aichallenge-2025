# Track/Cruise mixed-unit constraint acceptance

## Baseline

- Branch: `develop_july`
- Baseline commit: `a9c798c`
- Preserved unrelated change: `aichallenge/result-summary.json`

## Observed defects

`output/20260822-142549` retained exact five-state heading, but 78 of 7,505 Track/Cruise shadow
solutions still failed the unchanged physical wall certificate: 62 hard footprint contacts and 16
current-pose-to-horizon swept violations.

The first attempted physical-bound refinement exposed a more upstream acceptance defect. The QP
mixes metre-scale lateral rows with much larger course-progress rows. OSQP and the existing wrapper
used one global infinity-norm scale, so a large progress value could make a lateral error acceptable.
The extractor then added the observed global violation to its tolerance, making a bad solution
self-relaxing.

## Accepted scope

- Preserve violation and numerical tolerance for every constraint row in `SolveResult`.
- Evaluate five-state stage 1..N lateral box rows in their own metre scale.
- Reject Track/Cruise shadow output when a lateral row exceeds its own tolerance.
- Use the permitted lateral tolerance, never the observed violation, for pose extraction and wall
  certification.
- Keep the physical footprint certificate unchanged and authoritative.

## Rejected experiments

- A same-cycle second solve after exact-heading bound contraction: excessive solve time and almost
  no certified recovery.
- Reference-heading `e_y`-only preflight bounds: removed useful heading/lateral trade-off and
  increased QP failures.
- Linearized `e_y/e_psi` coupled rows: still failed to conservatively enclose the nonlinear body
  footprint while increasing solver and callback overruns.

## Non-scope

- No production authority, command, launch, parameter, clearance, or certificate change.
- No Track/Cruise authority promotion.
- No approximation of the nonlinear oriented footprint as a production hard constraint.
- Swept current-pose-to-first-stage reachability remains a later slice.

## Acceptance

- Mixed-unit failure-first tests prove a large progress row cannot relax a lateral row.
- Four-wrap single-car comparison remains `authority=shadow, selected=0`.
- Build and all package tests pass.
- Runtime cost returns to the exact-heading baseline after rejected constraint experiments are
  removed.

## Rollback

Rollback commit: `a9c798c`.

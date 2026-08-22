# Track/Cruise five-state pose contract

## Baseline

- Branch: `develop_july`
- Baseline commit: `1634c40`
- Preserved unrelated change: `aichallenge/result-summary.json`

## Problem

The five-state MPCC solves `e_y`, `e_lag`, `e_psi`, velocity and progress, but the Track/Cruise
shadow certificate first converts that solution to the legacy three-state layout and then derives
heading from adjacent lateral samples. The physical certificate therefore does not necessarily
validate the pose solved by the five-state formulation.

## Scope

- Define a typed five-state execution trajectory containing lateral, heading, velocity and progress.
- Preserve the exact stage-1..N `e_psi` values from the accepted primal.
- Allow the existing physical certificate to consume exact stage headings.
- Connect exact headings only to Track/Cruise shadow validation.
- Repeat the single-car shadow run and reclassify physical failures.

## Non-scope

- No production command or authority change.
- No physical wall bound refinement yet.
- No solver, cost, horizon or clearance tuning.
- No removal of the legacy conversion path.

## Acceptance

- Failure-first tests distinguish solved heading from a lateral-slope reconstruction.
- Malformed/non-finite extended state is rejected deterministically.
- Existing production validation callers retain their current derived-heading behavior.
- Dynamic run remains `authority=shadow, selected=0` and reports exact solution heading provenance.

## Rollback

Rollback commit: `1634c40`.

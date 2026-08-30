# Requirements

## Evidence boundary

- Baseline: `ed98ba7e audit(mpcc): reject normal-path terminal stop`
- Run: `output/20260830-200852`, Domain 1
- Frozen failure: decision 4017, ShiftOut side positive
- Evidence type: deterministic offline replay only

## Objective

Characterize the accepted seven-state Stop control sequence before choosing a
production architecture.  Determine whether a small deterministic control
lattice can plausibly represent it or whether a fresh seven-state solve is
required.

## Constraints

- Observation only; production authority and controller code do not change.
- No worker, Store, fallback, timeout, tolerance or parameter change.
- The frozen world, model, bounds, exact wall proof and dynamic proof remain
  identical.
- Do not infer a lattice from state trajectories alone; inspect the exact
  acceleration and steering-rate controls owned by the solved artifact.

## Exit evidence

- acceleration and steering-rate extrema, means and saturation counts;
- steering-rate sign-change count and per-stage sequence;
- stopping elapsed time, distance/progress and exact wall reserve;
- explicit recommendation: finite control lattice or asynchronous seven-state
  Stop artifact.

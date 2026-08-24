# Audit

## Baseline observation

`output/20260824-172712/d1/autoware.log` repeatedly reported cold
extended-Overtake failures whose worst residual was row 270, the stage-zero
curvature-rate constraint.

## First observation run

- Run: `output/20260824-174555/d1/autoware.log`
- The stage-zero curvature input box and rate interval overlapped in every
  captured failure. No `first_kappa_feasible=0` failure was observed.
- Representative overlap:
  - input box: `[-0.353141, 0.353141] rad/m`
  - rate interval: `[-0.213784, -0.181582] rad/m`
  - exact steering-coordinate interval:
    `[-0.214591, -0.180902] rad/m`
- Therefore an empty curvature intersection is refuted.
- Later failures showed `x0_ey=2.74 m` while the first future lateral tube was
  `[1.22, 1.62] m`. The horizon resolver currently proves only that each tube
  is wide enough; it does not prove that the fixed current state can reach the
  next tube through the QP dynamics.

## Revised causal hypothesis

Row 270 is a downstream residual, not yet the root. The stronger upstream
hypothesis is a temporally discontinuous lateral tube admitted as a complete
horizon. OSQP then compromises between impossible dynamics/tube constraints
and the tight first curvature row until the iteration limit.

The next observation records state-zero containment and the exact forced
stage-one lateral state. In the extended Euler model the first lateral state
has no direct input term, so this check is a deterministic feasibility proof,
not a heuristic rollout.

## Second observation run

- Run: `output/20260824-175458/d1/autoware.log`
- Every captured failure had the measured state zero inside its equality/box
  and the dynamics-forced first lateral state inside the first future tube.
- The first lateral input sensitivity was exactly zero, confirming that the
  diagnostic was the relevant hard feasibility test.
- Row 270 remained the dominant residual across Overtake, Track/Cruise and
  Rejoin while its physical intersection remained non-empty.

## Root cause

The stage-zero curvature-rate requirement had two numerical owners: the
ordinary input box and an additional unary rate row on the same scalar. This
does not change the physical feasible set, but creates non-unique dual
ownership at the most tightly constrained first input. Across formulations it
caused slow or failed dual convergence and made row 270 the recurring residual.

The producer repair intersects the exact steering-coordinate reachable range
into the stage-zero curvature input box. The redundant row remains only as an
unbounded structural placeholder, preserving the certified warm-start dual
layout. Inter-stage curvature-rate constraints remain active.

## Post-repair dynamic evidence

- Run: `output/20260824-180631/d1/autoware.log`
- Duration represented in the log: about 72.5 seconds.
- `failed_iterate_row=270`: 0 occurrences.
- All `failed_iterate_row`: 2 occurrences:
  - row 271: one legitimate stage-one interstage curvature-rate residual.
  - row 211: one stage-zero curvature input-box residual in the Follow worker.
- The producer did not report an empty first-curvature intersection.
- Both vehicle controllers reported zero callback overruns in the captured
  periodic runtime summaries.

The observation confirms the specific row-270 defect: the repeated residual
disappeared after removing the duplicate active owner. It does not claim that
all OSQP convergence problems are solved. The two remaining failures now have
distinct physical owners and remain visible for a later Slice; this Slice does
not hide them with solver tuning or fallback logic.

# Design

## Observed causal chain

1. Return obtained a certified seven-state solution immediately after
   `Pass -> Return`.
2. At lower actual speed, `resolve_reachable_temporal_horizon()` truncated the
   spatial 1 m reference from 20 stages to 4 stages.
3. The refined QP then required the terminal lateral state to enter the wall
   corridor reached by the optimized progress within roughly 0.36 s.
4. OSQP stopped at maximum iterations with row 63 outside its lower bound.
5. Retained authority later expired and Emergency became the only publisher.

For a seven-state, four-step problem, row 63 is the stage-4 lateral state box:

`state_values=7*(4+1)=35`, `row-35=28`, `28/7=4`, `28%7=0`.

## Independent feasibility check

`linear_feasibility.py` reconstructs only the immutable hard constraints from
the recorded assembly request:

- initial-state and affine-dynamics equalities;
- recorded state/input boxes;
- steering-rate prefix;
- progress-aligned wall slabs;
- swept lateral wall rows;
- supported dynamic-obstacle rows.

It solves a zero-objective LP using SciPy HiGHS. This does not replace or tune
the production QP. It answers whether the exact convex feasible set recorded in
the snapshot is empty independently of OSQP and its quadratic objective.

The reconstructed LP is infeasible. Its minimum common relaxation is about
0.0130. The conflicts are concentrated in the stage-4 lateral/heading boxes,
the progress/lag trust region and the recorded acceleration/steering bounds.

`nonlinear_feasibility.py` then applies the recorded bounds to the exact
nonlinear seven-state model. Global search followed by SLSQP also finds no
feasible four-step trajectory; the best minimum constraint margin is about
-0.0156. The failure therefore survives removal of the single-SQP affine
approximation.

## Decision rule

- LP infeasible: investigate physical infeasibility or conflicting constraint
  ownership before changing the solver.
- LP feasible but exact proof cannot accept an independently reconstructed
  trajectory: model/certificate mismatch.
- LP feasible and physical proof accepts: OSQP numerical/scheduling defect.
- A full current-world temporal Return bundle succeeds while the four-step
  request fails: lifecycle/horizon construction defect.

The independent feasible point cannot be passed to the production physical
proof because neither independent solve has a feasible point. This is not a
missing proof integration: the recorded four-stage feasible set itself is
empty.

## External comparison

The local upper-rank log keeps `N=20, dt=0.12` (2.4 s) even at standstill.
Canonical MPCC references optimize progress as a state on a time horizon; the
obstacle corridor is updated around the receding solution rather than shrinking
the optimization horizon to preserve a fixed spatial sampling.

The same pattern appears in the compared implementations:

- the local upper-rank log uses a fixed 2.4 s horizon;
- TUD-AMR `mpc_planner` keeps a fixed temporal discretization and evaluates
  topology alternatives asynchronously;
- Liniger MPCC optimizes path progress as a state over a receding time horizon;
- T-MPC++ evaluates multiple homotopy trajectories instead of shortening the
  time available to complete one.

## Earliest violated invariant

`resolve_reachable_temporal_horizon()` used current speed and maximum
acceleration to truncate the number of temporal stages. That made stage count
simultaneously mean:

1. temporal preview available to the controller; and
2. spatial reference samples deemed reachable by a scalar speed estimate.

This ownership is invalid for the current seven-state formulation. Progress
`theta` and lag already represent reachable spatial advance, and physical wall
refinement rebuilds the corridor at the progress selected by the first solve.
Reducing `N` therefore removed time needed for lateral Return without removing
the terminal Return requirement.

## Repair alternatives

1. Extend retained authority or add a Return grace period. Rejected: it hides
   the empty fresh request and adds lifecycle state.
2. Loosen solver settings, trust regions or wall clearance. Rejected: both
   linear and nonlinear feasibility checks fail before the objective matters.
3. Special-case a minimum Return horizon. Rejected: it preserves the invalid
   dual meaning of `N` and creates another phase-dependent patch.
4. Preserve the configured temporal horizon for every phase and let optimized
   progress express reachability. Selected: it restores one meaning for `N`,
   matches the formulation and removes the causative policy rather than adding
   a fallback.

## Implemented repair

- Removed the reachable-horizon policy, types and phase reason names.
- Removed its call from extended seven-state problem construction.
- Removed the tests that encoded speed-based temporal truncation.
- Kept the existing independently certified lateral-tracking prefix limit.

No authority, lease, fallback, solver setting, weight, wall margin or opponent
clearance changed. Frozen snapshots remain immutable, so the old snapshot still
correctly proves the old four-stage request infeasible; dynamic acceptance must
show that production now constructs a full-horizon Return request.

## Dynamic evidence

Two unchanged `make dev2` trials were run after the repair.

- `output/20260828-170302` reached the common extended-problem builder during
  ShiftOut. Its frozen dynamic-obstacle refinement snapshot records
  `horizon_steps: 20`; the removed speed-based truncation is absent at runtime.
- That run entered an unrelated ShiftOut refinement failure and Recovery before
  Return.
- `output/20260828-170636` did not obtain an executable overtake candidate and
  later lost Cruise/Follow authority before Return.

Neither run is valid evidence of Return completion or regression. Repeating the
same nondeterministic trial after two upstream failures would not test the
repaired condition. Return uses the same extended-problem builder and there is
no remaining phase-specific horizon reduction, so the repair is accepted as a
structural root-cause correction with Return dynamic completion explicitly
carried as an integration-gate observation, not silently marked passed.

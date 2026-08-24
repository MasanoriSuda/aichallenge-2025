# Design

## Root cause

Behavior intent and canonical production authority are currently changed in
one step, while the Follow producer is asynchronous.  On the first Follow
cycle the new problem can only be submitted after the current output is
resolved.  Therefore the consumer is already exclusive Follow authority while
its producer is necessarily still pending.  An older Follow episode may be
present in the retained store, but current-world and steering-history proof
correctly reject it when it is no longer executable.

The invariant that is missing is:

> A requested intent acquires production authority only together with a
> current executable canonical command.

## Options considered

1. Relax continuity or clamp the stale command: rejected because it separates
   the command from its immutable solution and hides the handoff defect.
2. Publish legacy/racing-line control for one cycle: rejected because it
   reintroduces a second normal authority at the migration boundary.
3. Keep Emergency as normal entry behavior: rejected because the producer gap
   is deterministic, not a physical emergency.
4. Run the same canonical Follow producer synchronously only for admission:
   selected.  It produces Gate-A evidence in the transition cycle; subsequent
   cycles continue through the asynchronous producer and retained proof.

## Repair

1. Extend the pure Follow production policy with a distinct
   `SolveTransitionAdmission` action.
2. Track only the intent of the last successfully published canonical normal
   command.  Emergency or legacy output cannot manufacture this evidence.
3. If Follow is requested and async/retained selection is unavailable:
   - when the last published intent is not Follow, run the existing fresh
     Follow evaluator on the current snapshot;
   - run the existing current-world retained evaluator over that exact fresh
     plan;
   - store and publish it only if the complete canonical selection succeeds;
   - otherwise use the existing Emergency supervisor.
4. If the last successfully published intent is already Follow, do not run the
   transition path.  Missing authority is a steady-state defect and remains
   fail closed.
5. Protect the shared Follow solver context with one lifecycle mutex so worker
   and transition admission cannot solve concurrently.
6. Carry the already-defined coherent-front evidence into the authority
   request.  A Follow action without that evidence resolves to Track/Cruise;
   the Behavior label is retained for diagnostics but cannot elevate
   production authority.

This is an atomic Gate-A completion, not a fallback: both paths use the same
problem formulation, solver, plan schema, physical certificate, selector, and
publisher.

## Non-scope

- Overtake target-release policy;
- target horizon availability;
- Overtake QP convergence;
- tactical side selection;
- parameter tuning.

## Validation

- Focused build: passed.
- Package tests: 40/40 targets, 1798 tests, 0 failures.
- Full `make autoware-build`: 25 packages passed.
- Final dynamic run: `output/20260824-222801`.
  - no-front Follow was normalized to Cruise and published a certified Cruise
    command instead of Emergency;
  - three valid Follow entries completed atomic admission in the same cycle;
  - transition admission rejects: zero;
  - `invalid-progress-evolution`: zero.

The run also isolated a separate steady-state Follow defect: six later cycles
lost canonical authority after Follow was already established (five current
target-horizon proof losses and one steering-continuity rejection).  Those are
not transition-admission failures and remain the next root-cause Slice.

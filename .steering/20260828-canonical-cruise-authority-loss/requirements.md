# Requirements

## Objective

Determine why the canonical seven-state Track/Cruise authority becomes
unavailable immediately after startup and during normal intent transitions,
preventing the dynamic ShiftOut acceptance run. Repair the upstream contract
without adding a fallback, lease, grace period, timeout or solver relaxation.

## Frozen evidence

Use `output/20260828-235049` as the immutable failing run. Both domains stopped
before a useful ShiftOut evaluation. Representative failures include:

- proposed current intent rejected as `intent-mismatch`;
- the previous published intent retained briefly, then rejected as
  `continuation-rejected`, `steering-unreachable` or `dynamic-path-blocked`;
- no current-world normal authority, followed by canonical emergency output;
- offline replay of some captured QPs failing, while one captured warm-start
  QP solves offline, so solver infeasibility alone is not yet established.

## Constraints

- Preserve production authority and the seven-state formulation.
- Do not change OSQP settings, wall/vehicle clearance, cadence or tuning.
- Do not add an intent resume rule, lease, grace period, timeout or fallback.
- Keep the unfinished late ShiftOut candidate isolated and unchanged while
  investigating this upstream blocker.
- Separate actuation-state infeasibility from asynchronous lifecycle gaps.

## Exit criteria

- Freeze the first accepted-to-unavailable Track/Cruise boundary per domain.
- Trace observation, actuation origin, submission, worker result, certified
  store, executed store, retained proof and atomic admission at that boundary.
- Classify the failure as physical/model infeasibility, solver limitation,
  scheduling/lifecycle defect or certificate mismatch.
- Any production change maps one-to-one to the proved root cause and has a
  focused regression test.
- Build and full package tests pass.
- `make dev2` demonstrates sustained normal Track/Cruise authority and reaches
  a meaningful dynamic Overtake evaluation.

# Requirements

## Objective

Remove the persistent Mission wall-clock budget from normal production
authority. The budget may remain diagnostic, but it must not force Return,
Recovery, side retry blocking, or Mission retention changes while the current
certificate pipeline owns execution.

## Frozen evidence

Run: `output/20260831-171209/d1/autoware.log`

- sequence 1161 crossed the canonical publisher as an exact current-world
  ShiftOut Bundle at decision 1774/1775.
- the same target/generation continued with valid normal authority.
- at 15.02 seconds, `same-target Mission total budget expired` changed the FSM
  from ShiftOut to Recovery despite no hard fault.
- decisions 1796-1798 then reported `phase=Recovery` while publishing the
  retained ShiftOut artifact, including a multiple-lateral-authority conflict.

The observed failure is authority/lifecycle mismatch, not solver, wall, target,
or physical infeasibility.

## Constraints

- Do not change the configured 15 s value.
- Do not add or extend a timeout, grace, lease, fallback, clearance, or solver
  tolerance.
- Rear-clear and exact Return admission continue to own normal completion.
- Hard faults, certified Stop, and external Emergency remain unchanged.
- Preserve budget telemetry so the old policy can be compared without owning
  commands or phase mutation.

## Acceptance

- A Mission budget observation cannot call `begin_validated_return`, arm a
  side retry block, forbid retention, or transition to Recovery.
- Static authority tests make this non-authority boundary explicit.
- Build and all package tests pass.
- Dynamic run does not show `same-target Mission total budget expired` followed
  by Recovery or a ShiftOut/Recovery phase-intent conflict.


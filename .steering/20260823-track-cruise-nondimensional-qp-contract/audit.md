# Audit result: rejected

## Static evidence

- Failure-first tests initially failed because the solver adapter and the
  five-state builder had no coherent variable/row scaling contract.
- The candidate implemented the exact coordinate mapping described in
  `design.md`, including physical primal/dual warm-start round trips.
- Focused tests passed.
- `make autoware-build` passed for all 25 packages.
- The complete package suite passed: 1608 tests, 0 errors, 0 failures.

## Dynamic falsification

Run: `output/20260823-112927`

The candidate was enabled only for canonical Track/Cruise and was observed as
`nondimensional=1`.  The run was stopped at the short gate; no six-lap test was
performed.

Observed during roughly one minute of active Track/Cruise:

- 8 `execution-primal-reject` outcomes:
  - 7 `virtual-progress-speed`, stage 0;
  - 1 `acceleration`, stage 14.
- 10 canonical Track/Cruise emergency-stop publications.
- 5 physical-certificate rejects.
- 3 current-pose hard-wall-contact observations.
- 1 swept-wall-path rejection.
- 0 solve failures or maximum-iteration failures.
- 0 control callback overruns.

The progress-speed violations were physical values from about `-0.0010` to
`-0.0028 m/s` against a physical tolerance near `0.0010 m/s`.  Each rejected
fresh result lacked a reusable retained proof and therefore caused a one-cycle
`-3.0 m/s^2` emergency command.

## Root cause of candidate failure

The coordinate transform preserves the exact feasible set and objective, but
OSQP's absolute stopping tolerance is not invariant under that transform.
Scaling a row by the reciprocal characteristic physical magnitude makes a
fixed solver-space `eps_abs` correspond to a larger physical error for
dimensions whose characteristic scale exceeds one.  The downstream physical
certificate correctly retains its original physical tolerance and therefore
continues to reject solutions that OSQP calls solved.

This is why an algebraically coherent transform can pass equivalence tests yet
still fail the production acceptance contract.  Relaxing the downstream
certificate, retuning `eps_abs`, adding a clamp, or retaining the candidate
behind a flag would only hide the mismatch and is prohibited by this Slice.

The wall evidence is not sufficient to claim that scaling alone created every
contact—the unscaled baseline also contains wall-certificate events—but it is
sufficient to fail the zero-contact short gate.  The emergency discontinuities
also remained, so this candidate did not remove the control hazard it was meant
to eliminate.

## Decision

Rejected.  All production and test code for nondimensionalization was removed.
Only this audit remains.  Do not retry bound-derived scaling unless solver
termination and physical certification are expressed by one provably
equivalent row-wise contract; the previously rejected row-normalization and
scaled-termination variants already show why partial versions are unsafe.


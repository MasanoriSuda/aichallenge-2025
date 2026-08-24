# Audit

## Data flow

```text
physical wall/target corridor
-> three-state lateral bounds
-> five-state QP hard bounds
-> nominal predicted states
-> immutable canonical plan with original physical corridor
-> elapsed-time retained corridor
-> current-world proof
```

## Root cause

The five-state QP receives the physical lateral corridor as its hard bound.
`extended_wall_tracking_reference_reserve_m` moves only the soft lateral
reference.  Competing progress, heading, curvature and velocity costs may
therefore place the optimized state closer to the physical bound than the
reserve needed for normal tracking and async reuse.

The immutable plan stores only the physical corridor and has no typed proof
that its nominal states retained a tracking tube.  Current-world proof then
correctly rejects the real vehicle as soon as a small tracking mismatch crosses
that physical bound.  Intermittent fresh solve availability turns this into
normal/Emergency authority chattering.

## Dynamic counterexample during implementation

The first implementation contracted state zero as well as future states. A
bounded `dev2` run (`output/20260824-142025`) repeatedly rejected both tactical
branches before solve with `lateral tracking tube unavailable at state 0` and
never entered an Overtake mission. This falsified the assumption that reserve
could be demanded from the already-observed initial equality. The corrected
contract requires reserve only from states the optimizer can choose (`1..N`);
state zero remains subject to the unmodified physical corridor.

The next dynamic run (`output/20260824-143403`) reached ShiftOut, but exposed a
second lineage break: the adopted pre-entry stored plan reported
`tracking_tube=0.000m`. Pre-entry left/right workers solve before
`progress_execution_context_active` becomes true, then seal a prospective
ShiftOut/Pass intent. Deriving bounds from FSM activity therefore gave the QP
one contract and the canonical plan another lifecycle. The corrected design
derives both the hard tube and the bounds schema from the prospective/current
canonical intent supplied before solve.

The intent-corrected run (`output/20260824-144324`) proved that the nominal
trajectory no longer consumed the configured reserve: at its first retained
authority break the expected state still had `0.286 m` of physical reserve.
The measured control state had nevertheless drifted `0.296 m` laterally while
the tactical worker reported `no complete or receding branch candidate` and
the old plan remained the only normal candidate. The physical proof correctly
rejected that state. This separates the repaired defect from the next one:
tracking reserve must be a hard plan contract, but a hard tube cannot replace
feedback or continuous receding candidate production.

The final bounded run (`output/20260824-145739`) also confirmed retained-plan
telemetry and lineage. Both fresh and stored retained selections reported
`tracking_tube=0.150m`; before the first rejection the nominal expected state
retained at least `0.170 m` of physical reserve. After fresh candidate supply
stopped, a roughly `1.75 s` old stored path accumulated about `0.186 m` of
lateral tracking error and crossed the physical bound by `0.015 m`. Emergency
then became the sole authority. No tolerance, lease or fallback can repair
that producer discontinuity.

## Contributor and downstream effects

- Contributor: Overtake solves are slower and occasionally reach maximum
  iterations, exposing stored-plan reuse more often than Follow.
- Mask/amplifier: explicit Emergency braking is correct fail-closed behavior,
  but alternating fresh and Emergency cycles enlarge the state mismatch.
- Downstream symptoms: progress discontinuity and cursor expiry occur after
  the first corridor rejection.
- Separate next root cause: active and pre-entry tactical cycles frequently
  return `no complete or receding branch candidate`; a stored immutable plan
  is then reused open-loop until physical proof fails.

## Rejected fixes

- Increasing `lateral_tolerance_m`: weakens the physical corridor proof.
- Reducing wall clearance: changes safety policy and does not repair lineage.
- Reusing by age/grace: executes an uncertified state.
- Retrying or holding a legacy command: adds another normal authority.
- Treating the soft reference as proof without changing QP bounds: the solver
  is still free to consume the reserve.

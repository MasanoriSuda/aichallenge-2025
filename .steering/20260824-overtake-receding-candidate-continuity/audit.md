# Audit

## Observed chain

```text
active ShiftOut
-> rolling Frenet-DP path refresh accepted
-> current legacy stage corridor rebuilt
-> five-state canonical build requests tube for all N future states
-> a later stage cannot carry the tube
-> no fresh canonical plan is published
-> stored immutable plan remains the only normal candidate
-> actual state drifts from its old nominal trajectory
-> current-world proof rejects initial corridor
-> Emergency authority
```

## Current evidence

- `OvertakeLine DP execution rolling refresh ... source=fresh_optimizer`
  proves tactical path generation did not stop.
- `Overtake canonical worker ... build-reject/extended MPCC lateral tracking
  tube unavailable at state 18` proves canonical conversion stopped before
  solve.
- `tracking_tube=0.150m` and expected reserve `0.170m` prove the previously
  sealed plan obeyed its nominal contract.
- The following `initial-corridor-violation` therefore describes execution of
  an aging plan, not nominal reserve consumption.

## Hypotheses

### H1: canonical horizon exceeds the certified receding prefix

- Support: tactical rolling refresh is accepted while a late canonical stage
  rejects before solve.
- Refutation: the failing stage lies inside the exact tactical certificate and
  the certificate claims at least 0.15 m reserve on both sides there.
- Required observation: failing physical lower/upper/width, stage distance,
  candidate certified distance and intent.
- Confidence: high.

### H2: physical corridor legitimately becomes too narrow inside the immediate
execution prefix

- Support: runtime stage-corridor telemetry reports a small minimum corridor.
- Refutation: failing stage lies beyond the prefix which can be consumed before
  the next worker update.
- Required observation: stage time/distance relative to canonical valid-until.
- Confidence: medium.

### H3: side/tactical identity is lost between DP refresh and canonical worker

- Support: active dual telemetry sometimes reports the opposite branch as
  `invalid branch side` and the committed branch as absent.
- Refutation: the canonical worker receives the same side, generation, path and
  bounds schema as the accepted refresh.
- Required observation: candidate source, side, generation and bounds identity.
- Confidence: medium.

## Rejected symptom fixes

- increase retained plan age;
- reduce reserve or wall clearance;
- ignore the failing late stage;
- keep the last steering command as a new authority;
- force opposite-side entry without a current physical certificate.

## Conclusion

H1 is confirmed.  Run `output/20260824-154559` produced nine fresh canonical
Overtake plans after the horizon was bounded to the last state which could
carry the unchanged 0.15 m tracking reserve.  The previous state-18 build
rejection therefore came from appending an unproved configured-horizon tail,
not from loss of the immediate execution prefix.

The same audit found a second instance of the same ownership defect in the
pre-entry left/right evaluator.  `build_extended_progress_problem()` could
return a shortened QP, while normalization, exact wall proof, terminal metrics
and the sealed problem context still used the caller's configured `N`.  A
valid bounded solution was consequently read with the wrong layout and could
never become the already-solved pre-entry canonical artifact.

H3 is rejected for the canonical worker path: target, side, generation,
geometry fingerprint and effective horizon are sealed from one snapshot.  The
opposite branch may still legitimately be absent from a tactical snapshot;
that is not the producer discontinuity observed here.

## Separate exposed failure

The bounded-prefix repair exposed a later, independent active-ShiftOut failure:

```text
fresh canonical plans published
-> about 1.0 s of ShiftOut execution
-> OSQP maximum iterations
-> worst final-iterate row = first curvature-rate constraint
-> retained plan eventually fails current-world initial corridor proof
```

This does not refute H1.  The failing QP had the full 20-stage horizon and its
first curvature-rate row was violated by the failed final iterate.  Whether
that represents an empty current-steering/tube reachability intersection or a
numerically unconverged but feasible QP is not yet proven.  An observation-only
diagnostic now reports the first curvature box, rate interval, their
intersection and the first two lateral tubes; no constraint or OSQP setting
was changed.

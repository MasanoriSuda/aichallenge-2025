# Results: Return lifecycle escape-hatch audit

## Observed failure

Run `output/20260831-015209`, decision 2856 entered Emergency Stop while the
Overtake phase was Return.  The retained source was sequence 2194, about
1.60 s after first publication.  Its immutable expected state differed from
the current control-origin state by 1.27 m and 0.219 rad, while its expected
speed was 1.41 m/s above the current speed.

The exact current-world normal publisher interval was still wall-clear.  The
terminal Stop contingency was not: its exact footprint first contacted the
wall near waypoint 218.  The independent Stop-successor shadow, which starts
from the fresh current state without extending the exhausted normal cursor,
also reported `static-path-blocked`.  The vehicle reached 0.17 m current wall
distance at decision 2871 and physical contact at decision 2876.

## Frozen A/B/C/D comparison

All arms used the decision-2856 immutable problem/world fingerprint and the
same exact wall, timed-obstacle and terminal Stop proof:

- A, captured persistent Return: main SQP solved; Stop wall proof rejected.
- A2, current target-bound Return: solved; the same Stop wall proof rejected.
- B, stateless current-world Return rebuild: solved; the same Stop wall proof
  rejected.
- C, eight independently generated smooth Return rejoin schedules: five main
  SQPs solved and reached the same Stop wall rejection; three were rejected by
  the solver.
- D, bounded three-step offline SQP continuation: no certified Bundle; solved
  descendants failed terminal intent construction and the remaining schedules
  retained their solver rejection.
- G, unchanged production Return population: solved; the same Stop wall proof
  rejected.
- H, wall restoration control: solved; the same Stop wall proof rejected.

The C/D search is bounded evidence, not a nonlinear infeasibility proof.
Consequently this snapshot must not be labelled globally physically
infeasible.

## Classification

At decision 2856, changing persistent Mission geometry to stateless or rough
Return geometry does not recover authority.  Main optimization succeeds but
the exact terminal physical certificate fails.  Under the requested exit
taxonomy this is **model/certificate mismatch**, not evidence for a Return
candidate-generation defect and not sufficient evidence for physical
infeasibility.

The failure is already downstream of the first useful correction point.  A
direct current-state Stop is also wall-blocked, so a production patch at
decision 2856 can only mask the symptom.  The next audit must find the last
predecessor decision that published normal authority while its successor
stoppability was about to be lost, and compare the certified predicted state
at the next publication origin with the state that actually arrived there.

## Existing contract and remaining question

`mpcc_rate_resolved_retained_revalidation` already implements the specified
partial transaction: one exact serialized normal publication interval followed
by a Stop suffix built from that interval endpoint.  Therefore the root cause
is not the simple absence of an endpoint Stop certificate.

What remains unproven is why a previously accepted transaction did not leave a
directly stoppable state at the next publication origin.  The candidate causes
are deliberately kept separate:

1. prediction-to-publication state join mismatch;
2. repeated rolling certification postponing the Stop boundary;
3. producer/emergency Stop models or actuation origins disagreeing;
4. the relevant predecessor used a full-suffix proof and therefore did not
   exercise the partial terminal contingency path.

No production authority, timeout, fallback, tolerance, clearance or parameter
was changed in this Slice.

## Next evidence gate

Before a production change, freeze the last accepted predecessor transaction
and record:

- normal publication endpoint state and pose;
- certified Stop initial state at that endpoint;
- next-cycle fresh control-origin state and pose;
- direct Stop trajectory from that fresh state;
- command, steering-response and observation/control timestamps;
- whether predecessor scope was full suffix or publisher-interval prefix.

Only after the first diverging invariant is deterministic may the old contract
branch be replaced.

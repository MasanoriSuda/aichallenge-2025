# Root-cause audit

## Observed phenomenon

The preceding certified-residual repair succeeds dynamically.  In domain 1 of
`output/20260826-111752`, decision 900 then exposes the next first race-start
failure.  Sequence 301 is solved and wall-certified, but current-world join
returns `steering-unreachable`.

## Causal graph

```text
measured steering at now
  -> bounded physical projection to now + 130 ms
  -> six-state state zero / exact wall proof: accepted
  -> artifact cursor at that future control origin: elapsed 0
  -> steering sampled at elapsed 0
  -> future physical state treated as immediate desired publication
  -> 25 ms desired-command reachability rejects
  -> atomic Track-to-Cruise admission unavailable
  -> explicit Emergency
```

## Classification

- Root: physical prediction time and desired publication time are represented
  by the same extracted steering value.
- Contributor: steering actuator lag makes future physical state and previous
  desired command visibly different at race start and curves.
- Mask: Emergency is correct fail-closed behavior but appears as unexplained
  braking.
- Recovery: not involved in the first failure.

## Competing hypotheses

- Wall/dynamic conflict: falsified by physical proof accepted and no blocker.
- QP failure: falsified by `solver=solved` and exact certification.
- Tight steering parameter: rejected as root because the wrong time sample is
  compared before any legitimate tuning decision.
- Publication-time substitution: directly supported by code and decision-900
  values; highest confidence.

## Implemented repair

- The execution artifact seals the exact publication interval used by its
  originating solver snapshot.
- Artifact validation proves that the interval is finite, positive, within the
  sealed horizon and sampleable on the exact steering sequence.
- Actuation extraction samples steering at `cursor elapsed + publication
  interval`; it no longer substitutes the prediction-origin physical state for
  the next desired publication.
- Physical steering initialization and last-published command continuity remain
  separate contracts.  No bound, margin, solver setting, timeout, lease or
  normal fallback changed.

## Static verification

- Failure-first test reproduced the old substitution: extracted steering was
  `0.100000 rad` while the certified next-publication value was
  `0.095554 rad`.
- The repaired test plus invalid zero/beyond-horizon interval cases pass.
- Seven directly affected test targets pass.
- The complete package suite passes: 51/51 tests.
- `make autoware-build` passes: 25 packages.

## Moving verification

`make dev2` produced `output/20260826-113945`.  Both domains entered moving
Cruise operation, production published certified six-state commands and the
preceding `command-rejected` signature remained absent.  Domain 1 sustained
periods of 81/81 production availability with zero callback overruns.

The corrected time contract exposes a distinct upstream defect under saturated
steering rate.  For example decision 1195 has a valid next-publication sample
of `-0.183706 rad`, but the previous published desired command is
`-0.132350 rad`, outside the 25 ms reachable interval.  Later examples have the
same typed signature.  The QP constrains steering rate from the future physical
prediction origin but does not constrain its first publication sample against
the previous desired command.  Retained revalidation therefore rejects the
plan correctly.  This is the failure-first input for the next formulation
Slice; it must not be hidden by relaxing revalidation or adding fallback.

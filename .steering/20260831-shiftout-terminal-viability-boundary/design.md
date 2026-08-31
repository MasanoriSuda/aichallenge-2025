# Design: ShiftOut terminal viability boundary

Use the immutable snapshot:

`output/20260831-151543/d1/mpcc_architecture_snapshots/`
`000000001838-f4293cb6492b3595-shiftout-side-positive-physical-proof-`
`terminal-contingency-unavailable/snapshot.yaml`

Run the existing observation-only architecture comparison without changing
production authority.  Correlate its arms with the runtime chronology:

1. last actually published certified ShiftOut command;
2. first loss of normal recursive terminal viability;
3. emergency Stop publication;
4. later actual-footprint wall-margin violation.

If all decision-1838 arms fail, do not infer physical impossibility until the
earliest previously viable decision is located.  The intended output is the
first causally actionable boundary, not another downstream exception.

## Frozen comparison result

The unchanged comparison was run against the decision-1838 snapshot.  The
persistent arm A, both stateless homotopies B, the rough/lattice arm C and the
offline/nonlinear arm D all failed to produce a certified maneuver.  The
production and wall-restoration comparison arms failed at the same exact wall
boundary.  A separate terminal-Stop comparison also failed: the seven-state
Stop and the 68-candidate control lattice both reached approximately zero
terminal velocity but intersected the exact wall certificate at sample 21.

This establishes only that decision 1838 was outside the represented recursive
viability set.  It does not establish that ShiftOut was physically infeasible
when accepted.  Normal authority was still accepted at decision 1837, one
control cycle earlier.

## Runtime chronology

- decision 1829 accepted and published certified ShiftOut artifact 248;
- decisions 1830--1837 retained normal authority;
- decision 1838 rejected the retained artifact because the rebuilt terminal
  Stop collided with the wall about 1.3 m ahead;
- the serialized publisher interval itself remained wall- and peer-clear;
- Emergency Stop was published at decision 1838;
- the FSM reported actual-footprint wall-margin violation only at decision
  1851, after normal authority had already been lost.

At decision 1838 the current-world join also reported 0.121 m position error,
0.056 rad yaw error and approximately 0.089 rad difference between measured
steering and the artifact's expected steering.  Existing logs do not record
the corresponding decision-1837 accepted proof, so they cannot distinguish a
one-cycle plant/model divergence from a course-chart discontinuity or an
artifact/publication identity error.

## Classification

The downstream Recovery is rejected as the root cause.  Candidate generation
and single-SQP limitations at decision 1838 are also rejected because A/B/C/D
all agree.  The current classification is **model/certificate boundary not yet
observable**: the last viable proof and first infeasible proof are not logged as
an immutable pair.  Production behavior must remain unchanged until that pair
is captured.

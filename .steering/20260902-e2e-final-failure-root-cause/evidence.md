# Evidence

## Timestamp correction

The existing motion report selected later recorder segments beginning around
281 s because they were the longest contiguous low-speed intervals.  d1 and d2
had already entered sustained low speed at 42.875 s and 43.511 s respectively.
All causal replay windows were therefore moved to ten seconds before those
first stops.

## Runtime composition

Before the first stop, offline composition and the nearest published steering
match to sub-microradian error.  Wheel versus fused speed has p95 absolute
difference below 0.11 m/s.  A composition or speed-topic regression is not the
first cause.

During the twenty seconds straddling the first stop:

- d1 spatial correction has mean `-0.0635 rad` and changes sign after the
  vehicle enters the trap;
- d2 spatial correction has mean `-0.3152 rad`, reaches `-0.6191 rad`, and the
  final command reaches the node limit `-0.64 rad`;
- the historical precontact teacher and v11 agree well before the precursor
  (`0.0180 / 0.0253 rad` mean absolute residual error for d1/d2), then diverge
  after physical clearance collapses.

The constant post-contact corrections observed in the first replay were
symptoms, not proof of the collision cause.

## Course-region comparison

d1/d2 incur wall penalties at race time 21.21/21.11 s after about 48 m.  Around
the same region, clean d3/d4 maintain roughly 3.3--3.8 m/s with 4--9 m frontal
clearance and continue through the turn.  d1/d2 frontal clearance collapses to
about 1.5--1.9 m while they approach behind the leading peers, then both stop
against the wall.  The frozen failure is therefore an interaction-dependent
lateral/longitudinal recoverability problem rather than a general checkpoint,
topic, inference or track-following failure.

## Recurrent A/B

`output/20260902-e2e-final-recurrent-024` enabled only the already admitted,
default-off recurrent correction at `+/-0.24 rad`; base/spatial artifacts,
acceleration, speed cap, distance safety and world were unchanged.  The run was
stopped after the first interaction and AWSIM wrote authoritative partial
results.

The candidate changed the failure rather than fixing it:

| Domain | Frozen packaged run | Recurrent A/B |
|---|---|---|
| d1 | wall at race 21.21 s; 48.9 m | crash at 16.45 s; 32.4 m; long stop |
| d2 | wall at 21.11 s; 48.3 m | first trap passed; 380.0 m; later wall |
| d3 | no penalty in full frozen run | new crash at 12.87 s and later wall |
| d4 | no penalty in full frozen run | one lap, later wall |

d1 remained at zero speed with front clearance about 1.64 m.  The existing
distance policy classified this as `slow-clearance` and continuously replaced
positive acceleration with zero, so lateral authority could not move the
stationary Ackermann vehicle out of the trap.  Recurrent authority itself had
finite output and no inference error, but introduced earlier crashes and is
rejected for production.

## Classification

- input topic or runtime composition defect: rejected by replay parity;
- general track-following defect: rejected by clean d3/d4 traversal;
- existing recurrent candidate as the fix: rejected by new d1/d3 crashes;
- remaining root: interaction-dependent recoverability, with both insufficient
  pre-contact policy coverage and a structural zero-speed `slow-clearance`
  deadlock.

Production authority remains unchanged.  The next Slice must compare bounded
speed-aware longitudinal policies offline before changing runtime.

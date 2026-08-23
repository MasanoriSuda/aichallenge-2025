# Audit: Track/Cruise Slice 3 closure

## Invalid run excluded

`output/20260823-121008` is not acceptance evidence.  Its runtime log contained
`Track/Cruise condensed shadow` even though that experiment had been removed
from the Git tree.  The workspace install artifact had not been rebuilt after
the experimental source deletion.

The run was stopped, `make autoware-build` completed successfully, and an
installed-artifact string scan proved the removed observer was absent before
the valid run began.  This source/install identity check is now a mandatory
dynamic-gate precondition.

## Valid six-lap evidence

Run: `output/20260823-121707`

Lap times:

| Lap | Time [s] |
|---:|---:|
| 1 | 46.456 |
| 2 | 43.560 |
| 3 | 43.515 |
| 4 | 44.400 |
| 5 | 43.285 |
| 6 | 42.885 |

The 285 one-second production summaries contain:

- eligible canonical cycles: 11,678;
- OSQP results returned: 11,628;
- strict execution-primal accepted: 11,002;
- physically certified and published fresh candidates: 11,002;
- fresh solve-unavailable cycles: 50;
- solved but semantic-boundary rejected cycles: 626;
- total fresh canonical unavailability: 676 cycles;
- certified coverage: 94.21% of eligible cycles.

Change-only outcome logs classify the observed semantic rejects primarily as
stage-zero curvature, followed by stage-zero acceleration, stage-one predicted
velocity and stage-zero virtual-progress speed.  This matches the previously
audited mixed-unit OSQP boundary defect.

The same run reports:

- callback overrun: 0;
- physical-certificate reject: 0;
- wall/contact event: 0;
- abrupt measured speed loss: 0;
- confirmed Stuck: 0;
- Reverse maneuver: 0;
- Recovery: 0;
- removed condensed-shadow observer: 0 occurrences.

The final command trace contains canonical fresh publication and explicit
`canonical-track-cruise-emergency-stop` outcomes.  It contains no retained
publication because the one-car run reports V2X `NoData`, so current-world
obstacle proof correctly fails closed rather than assuming an empty world.
It also contains no Track/Cruise legacy/three-state normal publication.

## Root-cause conclusion

The remaining defect is canonical solver availability, not authority
arbitration.  Strict semantic rejection and Emergency output are functioning
as designed.  Six laps complete without a physical or real-time regression,
and the old normal formulation never regains control.

The numerical defect is not declared fixed.  It is closed as an exhausted,
visible residual risk for this migration phase.  A future replacement must
start as a separate failure-first solver/formulation Slice and must outperform
this baseline without weakening certification.

## Decision

Accept Track/Cruise Slice 3 architecture migration.  Proceed to Slice 4 only
in audit/shadow mode.  Do not promote Follow/Hold/Stop authority until its own
dynamic evidence and explicit approval exist.

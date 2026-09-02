# Design

The teacher currently has three separate concepts:

1. `dynamic_distances()` computes a speed-dependent stopping/preview envelope;
2. `LidarPrecontactTeacher` selects one instantaneous angular gap;
3. side state prevents late homotopy flips.

The dynamic stop distance is not a longitudinal constraint.  It only expands
the trigger used by the instantaneous gap proposal.  Conversely, the proposal
does not contain a time-indexed trajectory, opponent prediction, swept vehicle
footprint or terminal successor.  Therefore the code can report a 21 m
stopping requirement while continuing through a 5 m frontal observation, but
cannot state why that continuation is physically viable.

The audit will replay the exact teacher and classify every scan into:

- clear of the dynamic stop envelope;
- inside the dynamic stop envelope with no lateral proposal;
- inside the envelope with a committed gap proposal;
- braking/zero/forward command;
- side acquisition, maintenance, switch and release.

It will also measure contiguous episodes and the final pre-stall prefix.  The
same counts are computed for certified successful runs.  If both successful
and failed runs routinely continue inside the stop envelope, that predicate is
rejected as a runtime fix.  If the failed episode cannot be separated using
the teacher's existing fields, the result is a candidate-representation defect:
an instantaneous polar gap is not an escape trajectory certificate.

No approximate footprint heuristic will be promoted from this report.  Such a
heuristic would add another unverified safety owner.  A later Slice may compare
a short-horizon lattice/polynomial teacher or MPC-labelled trajectory, but only
after this audit fixes the required observable contract.

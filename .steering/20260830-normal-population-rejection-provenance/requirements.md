# Requirements

## Objective

Make every failed current-world normal MPCC population evaluation observable
without changing production authority or relaxing any proof.

The frozen failure is decision 4120 in `output/20260830-104041/d1`:

- the vehicle was travelling at about 4.66 m/s;
- the requested intent changed from Cruise to Follow;
- the last published Cruise artifact, sequence 3422, was no longer
  progress-continuous;
- no fresh Follow authority existed;
- the controller emitted Emergency Stop and stuck recovery selected Reverse.

During the preceding two-second telemetry window 81 worker evaluations
completed but all 81 were counted as invalid mailbox results.  The candidate
generation failure reason was therefore lost.

## Constraints

- Do not change production authority selection.
- Do not add a lease, grace period, timeout or fallback.
- Do not change solver settings, clearance, speed or behavior thresholds.
- Preserve immutable problem and sequence identity.
- A rejected worker evaluation must never become executable authority.
- Record enough evidence to distinguish candidate generation, missing
  physical input and solver/proof rejection.

## Definition of Done

- Manually constructed worker rejections satisfy the mailbox result schema.
- Rejected results remain non-solved and non-executable.
- The exact rejection detail reaches existing worker failure telemetry.
- Focused tests, full package tests and build pass.
- A dynamic run exposes the upstream rejection which preceded authority loss.


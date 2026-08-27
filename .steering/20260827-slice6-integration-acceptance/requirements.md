# Requirements

## Objective

Close the Slice 6 integration Gate after every normal command owner has been
migrated to the canonical seven-state rate-resolved MPCC and the retired
three/five-state normal implementations have been physically removed.

This Slice validates integration quality; it does not add another controller,
fallback, lease, timeout, or tuning parameter.

## Scope

- Exercise Track/Cruise, Follow, ShiftOut, Pass, Return and Rejoin through the
  canonical production authority in multi-vehicle AWSIM runs.
- Confirm that Emergency and Recovery remain external supervisors and do not
  become alternate normal command producers.
- Audit fresh/retained current-world proof continuity, target lifecycle,
  branch provenance, callback tail and final serialized command origin.
- Identify the first violated invariant for any abnormal lap, stop, contact,
  authority hole or stale target transition.
- Repair only a demonstrated producer/consumer contract defect, with a
  failure-first regression test and deletion of any obsolete branch it makes
  redundant.

## Non-goals

- Parameter, wall-margin, speed, weight, horizon or solver tuning.
- Restoring a legacy MPC, three-state or five-state normal fallback.
- Adding a grace period merely to mask an authority discontinuity.
- Changing ROS topics, messages, Domain layout, evaluation schema or launch
  entry points.

## Dynamic acceptance

1. `make dev2` and, after that passes, `make dev3` start and produce commands
   without manual topic or source edits.
2. Every normal published command is attributed to the seven-state canonical
   authority; legacy normal/formulation-switch publication count is zero.
3. Track/Cruise and Follow are observed. Overtake must reach ShiftOut and, when
   the physical scene permits, Pass/Return without an unexplained authority
   hole. Unreached intents are recorded as missing coverage, not declared
   accepted.
4. No stale branch, wrong generation, unproved current stage, non-finite
   artifact, or current-stage wall/dynamic-obstacle violation is published.
5. No continuous 25 ms callback overrun episode. Isolated tail events are
   measured and attributed.
6. Any Emergency or Recovery episode has a preceding typed cause; it must not
   be the first unexplained event.
7. Generated JSON, log and MCAP files remain outside the commit.

## Definition of done

- The first failing invariant, if any, is deterministically reproduced.
- A structural repair and its failure-first test pass, or the existing system
  passes both dynamic Gates without production changes.
- Focused/full package tests and `make autoware-build` pass after any code
  change.
- Audit evidence states observed intent coverage and all remaining risks.

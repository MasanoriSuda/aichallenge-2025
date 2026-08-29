# Requirements: Stop-to-normal current-world bundle

## Objective

Remove the self-sustaining normal-authority hole reproduced in
`output/20260829-214906` without adding a resume rule, timeout, lease, grace
period, fallback, solver setting or clearance change.

The production comparison is frozen at `eb3b4fc2`.  The comparison arms are:

- A: persistent certified-plan execution and the existing seven-state SQP;
- B: a stateless current-world ManeuverBundle built from the same candidate
  controls and the existing exact nonlinear/current-world proof;
- C: the already implemented positive/negative current-world population;
- D: the existing offline nonlinear feasibility oracle.

## Frozen evidence

- Both domains eventually published external Stop and then retained it with
  `gate_a_attempted=0` while the proposed normal intent remained unavailable.
- D1 recorded 6,864 latest-state steering connections; 5,978 passed the
  unchanged nonlinear, wall, timed-obstacle and terminal-successor proof.
- D2 recorded 1,903 connections; 1,831 passed the same proof.
- D1 continued to certify fresh candidates while the old executed artifact
  cursor was exhausted.  A representative final window had a fresh candidate
  at cursor `0.10--0.135 s` whose only rejection was
  `steering-unreachable`; 73--75 of 81 connected continuations proved clear.
- D2 produced both positive and negative Cruise candidates and 38 exact-clear
  dynamic results, so a missing side population is not the Stop latch cause.

## Invariants

- Emergency Stop remains an external supervisor and owns zero speed and
  braking when no normal proof exists.
- A latest-state connection may publish only after the unchanged exact
  nonlinear, wall, timed dynamic-obstacle, Follow-gap and terminal-successor
  proof accepts it.
- The connected result is a stateless current-world bundle.  It must not mark
  the source asynchronous plan as executed, because its first serialized
  steering command differs from that source artifact.
- The source plan remains immutable provenance; the current decision,
  observation generation, exact continuation and Stop suffix form the bundle
  certificate.
- One normal publisher and one seven-state formulation remain.
- No behavior parameter, physical tolerance or solver setting changes.

## Definition of done

- A deterministic regression reproduces an unreachable prepared steering
  whose exact current-world connection proves feasible.
- The result creates canonical authority and is explicitly tagged as a
  stateless current-world bundle.
- Publication of the bundle cannot promote the source plan into the executed
  persistent-plan ledger.
- Failed nonlinear/current-world proof remains `steering-unreachable` and
  cannot create authority.
- Build, full package tests and a bounded dynamic gate pass.

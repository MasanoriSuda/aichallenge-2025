# Requirements: bounded normal-avoidance population

## Objective

Repair the candidate-generation defect proven at frozen decision 2451 by
giving Cruise/Follow the same bounded, anytime current-world topology search
used by Overtake, while preserving neutral normal intent and the single
certified publisher.

## Causal link

The direct positive and negative current-world candidates both failed.  A
smooth negative schedule from the identical world produced a complete
ManeuverBundle.  Therefore the repair must expand the bounded candidate
population, not alter Stop, solver tolerances, clearances, Mission leases or
retained execution clocks.

## Invariants

- Direct side remains first and remains the fast path.
- Additional candidates are rebuilt from the same immutable current-world
  fingerprint.
- Cruise/Follow keeps `execution_side_sign=0` and does not acquire Overtake
  phase authority.
- A side branch publishes only after the existing SQP, exact trajectory,
  wall, dynamic and terminal Stop proofs all accept.
- Sibling work remains asynchronous and cannot delay the primary side.
- A fully certified depth-zero result is never replaced by a later failed
  refinement.

## Out of scope

- No parameter, clearance, horizon, timeout, lease, grace, fallback or Stop
  authority change.
- No decision-2451-specific condition.
- No retained lattice trajectory across world epochs.

## Definition of done

- Both normal sides own bounded direct plus derived smooth schedules.
- The primary evaluates its candidates in anytime order; the sibling does the
  same on its existing worker.
- Frozen decision 2451 produces a certified normal bundle in the production
  population replay.
- Full package build and tests pass.

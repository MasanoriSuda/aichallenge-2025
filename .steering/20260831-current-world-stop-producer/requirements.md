# Requirements

## Objective

Replace the stale published-artifact terminal Stop producer with a producer
whose immutable input is the current-world seven-state problem bound to the
actually serialized predecessor.

## Frozen failure

Run `output/20260831-122218/d1` lost ShiftOut authority at decision 1576.
The selected normal artifact was sequence 929.  Its ordinary continuation was
valid over the publisher interval, but its fixed terminal Stop hit the wall.
The live alternate Stop had been certified from the old solver epoch and was
rejected at the current decision as `steering-unreachable`.

The immutable failure snapshot is:

`output/20260831-122218/d1/mpcc_architecture_snapshots/`
`000000001576-6145eddcc3ca72fb-shiftout-side-positive-`
`physical-proof-terminal-contingency-unavailable/snapshot.yaml`

## Constraints

- Do not add a Mission resume rule, lease, grace period, timeout or fallback.
- Do not change solver tolerances, weights, horizon or clearances.
- Do not allow an old published normal trajectory to masquerade as a current
  Stop source merely because its tactical scope is unchanged.
- Preserve one canonical normal publisher and exact wall, dynamic and rest
  certificates.
- Remove the obsolete published-artifact Stop submission path in the same
  Slice.

## Definition of Done

- The Stop worker consumes the same immutable current-world snapshot created
  after serialized-predecessor binding as the normal asynchronous pipeline.
- A current-world maximum-braking seven-state Stop can be built without an
  already solved historical normal artifact.
- Latest-only scheduling remains bounded and observation-only until exact
  current-world join succeeds.
- Same-snapshot comparison, build and package tests pass.
- A dynamic run shows current-world Stop results are produced and either join
  authority or expose the next independently classified failure.

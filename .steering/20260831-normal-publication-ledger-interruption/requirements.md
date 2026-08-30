# Requirements: normal publication ledger interruption

## Objective

Prevent a certified normal MPCC artifact from advancing through control cycles
in which an external Stop or another non-normal authority actually crossed the
publisher boundary.

## Frozen evidence

- Baseline: `4cde17be`
- Run: `output/20260831-024939/d1`
- Overtake episode: target `d3`, side `-1`, ShiftOut -> Pass
- First terminal-contingency loss: decision `1269`
- Certified Stop-successor Bundle: decision `1279`, sequence `680`
- Unreachable resumed cursor: decision `1297`
- Wall-margin recovery: decision `1307`

## Invariants

- Only a command which actually crossed the publisher may advance a normal
  artifact's execution ledger.
- Publishing Stop, stuck Recovery, disabled-control output, or wall-hold output
  interrupts continuity of every previously executed normal artifact.
- An interrupted artifact may not later use `PublishedPlan` wall-clock age as
  evidence that its skipped controls were executed.
- Candidate certification remains monotonic. A later current-world candidate
  may regain normal authority only through the existing exact current-world
  actuation, wall, dynamic-obstacle, and recursive Stop proof.
- Do not change solver settings, timeouts, leases, grace periods, velocity
  limits, wall clearance, or production safety margins.

## Architecture comparison

The decision-1269 immutable snapshot was evaluated with the existing A/B/C/D
tool. Persistent A, stateless B, rough/lattice C, and bounded offline D all
failed to produce a certified Bundle from that already-diverged state. The
live system had nevertheless produced certified Stop successor sequence 680
at decision 1279, then alternated that normal artifact with external Stop.
The later failure is therefore classified as a scheduling/lifecycle defect,
not as evidence that a different Mission geometry or clearance value is the
root correction.

## Definition of done

- A non-normal publication invalidates the executed/published normal ledger.
- A candidate which did not cross the publisher is not falsely marked
  executed or cleared merely because another authority published.
- Unit tests cover Stop interruption and subsequent fresh candidate adoption.
- Static single-authority contracts and package tests pass.
- Dynamic logs contain no resumed `PublishedPlan` cursor spanning external
  Stop decisions for the same artifact identity.

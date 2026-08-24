# Requirements

## Objective

Remove the observation-only rate-resolved physical wall proof from the 40 Hz
control callback while preserving exactly the same physical input, certificate
strength and shadow-only authority boundary.

## Root cause

`output/20260825-041116` proved that the physical contract is complete, but the
swept-footprint calculation reached 10.485 ms on the control thread and two
callbacks exceeded their 25 ms budget. The defect is scheduling ownership, not
wall clearance, solver parameters or proof feasibility.

## Constraints

- No parameter, solver setting, behavior state or command change.
- No physical proof weakening or stage subsampling.
- No age-only acceptance: completed results must match artifact identity,
  intent, stage geometry and the captured current-world snapshot.
- Keep `authority=shadow, selected=0`.
- Do not connect the six-state artifact to the five-state canonical publisher.

## Definition of done

- Physical proof runs exclusively after the solve in the existing latest-only
  rate-resolved pipeline worker; no second 40 Hz worker is added.
- Control-thread work is bounded to immutable snapshot construction,
  non-blocking submission and non-blocking result consumption.
- Unit/source-contract tests cover provenance, stale rejection and the absence
  of synchronous proof execution.
- Full build/test passes.
- A bounded `make dev2` run produces accepted physical proofs without adding
  callback overruns attributable to the shadow pipeline. Existing production
  solver/certificate overruns are recorded as a separate typed blocker rather
  than hidden by this slice.

## Rejected intermediate architecture

`output/20260825-044053` used a second latest-only physical-proof worker. The
certificate pipeline was semantically correct (2455 consumed results, 2277
current-semantic results, no worker rejection, 6.834 ms maximum proof time),
but D1 produced 21 observation-only callback overruns at 54--63 ms versus two
overruns and 26.993 ms maximum in synchronous baseline
`output/20260825-041116`. This falsified the assumption that a second worker was
free of control scheduling cost. The implementation must use one serialized
solver/certificate worker instead.

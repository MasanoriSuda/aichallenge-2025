# Requirements

## Baseline

- `0597a685 refactor(mpcc): isolate terminal stop lattice source`
- Production authority, certified Store, sole publisher, legacy Stop successor,
  parameters and physical tolerances remain frozen.

## Objective

Collect live evidence that the bounded seven-state Stop lattice can be built
from the exact normal candidate epoch selected by the asynchronous MPCC
worker, without blocking the 40 Hz callback or delaying normal Store admission.

## Constraints

- Only ShiftOut and Pass are sampled in this Slice.
- Use a separate latest-only worker and a private solver context.
- The selected normal plan must enter the existing Store before Stop shadow
  submission.
- Every Stop candidate must pass the unchanged exact nonlinear trajectory,
  wall, all-peer dynamic and certified-plan validation chain.
- The Stop result may enter only an observation mailbox; it may not enter the
  certified Store, production adapter or publisher.
- No fallback, lease, grace period, timeout, clearance, solver tolerance or
  Mission transition change.

## Exit gate

- static audit proves no Store/publisher/production callsite;
- unit tests cover identity, deterministic population and mailbox monotonicity;
- full package tests pass;
- live log reports submission replacement, acceptance, age, attempts and
  solve/total timing before any production promotion is considered.

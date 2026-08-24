# Root-cause audit

## Observation

The rate-resolved worker solves, extracts one complete artifact and performs
the exact swept-footprint proof serially.  Solver and wall outcomes are then
published independently and consumed independently by the control callback.

## Causal chain

1. Numerical success owns the immutable six-state artifact.
2. Physical success owns the sealed current-pose/course-frame identity.
3. No retained object owns both facts atomically.
4. A future fresh/retained selector would have to infer their relationship
   from separate latest-result streams.
5. That inferred join cannot provide the one-to-one provenance required to
   delete the five-state normal fallback.

## Root cause

The producer boundary ends before a complete executable certificate exists.
The missing invariant is not a solver tolerance or wall margin; it is atomic
ownership of the exact artifact and its exact accepted physical proof.

## Rejected alternatives

- Comparing only sequence or age at consumption: incomplete identity and
  age-only acceptance.
- Copying the artifact into the five-state plan store: changes formulation
  semantics and loses steering-rate ownership.
- Promoting a fresh result now: leaves no same-formulation retained path and
  turns transient async unavailability into Emergency stops.

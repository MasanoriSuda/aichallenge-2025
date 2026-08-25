# Design

## Authority flow

```text
current Track/Cruise observation
  -> retained six-state current-world proof
  -> rate-resolved production adapter
     -> complete canonical problem/solution identity
     -> exact first executable actuation
     -> remaining world prediction
  -> CanonicalNormalCommand
  -> unchanged Emergency/Recovery supervisor
  -> final publisher and execution trace
```

The next asynchronous request is built independently from the five-state
solver. It is sealed before publication, then its state-zero steering is bound
to the command selected in this cycle.

## Canonical command contract

`CanonicalNormalCommand` is a formulation-tagged publisher contract, not a
five-state mathematical object. Its validation accepts only the two explicit
canonical formulations during migration:

- `VelocityProgress5State` for Follow/Overtake/Rejoin owners not changed here;
- `VelocitySteeringProgress6State` for Track/Cruise.

Legacy three-state and direct paths remain noncanonical. The final trace must
validate the command identity for either supported canonical formulation.

## Six-state production adapter

The adapter consumes an accepted retained proof and constructs:

- the unchanged sealed source context;
- a certified solution identity derived from the immutable artifact and its
  accepted physical/current-world proof;
- the exact sampled speed, acceleration, steering and virtual progress rate;
- the remaining steering/speed horizon;
- the remaining world-coordinate prediction reconstructed from the exact
  physical snapshot.

It cannot solve, clamp, select a side, weaken a certificate or publish.

## Exact physical-bound ownership

The persistent OSQP certificate permits a bounded row residual, while the
publisher contract requires the executed physical actuation to lie inside the
actuator envelope exactly. The solver-facing velocity, acceleration,
steering-rate and virtual-progress rows are therefore inset by the maximum
residual already permitted by the existing physical OSQP tolerance. The
physical bounds themselves remain unchanged and are retained in the execution
artifact for exact validation.

The final canonical publisher does not clamp or otherwise post-process a
certified command. If a solved input cannot satisfy the original physical
envelope exactly, artifact construction fails closed. Recovery and
noncanonical paths keep their existing output clamps.

## Failure behavior

If any identity join, physical evidence, cursor, actuation or world prediction
is incomplete, the adapter rejects the authority. Track/Cruise then uses the
existing explicit canonical EmergencyStop. It does not call the five-state
solver as a fallback.

## Removal in the same Slice

The Track/Cruise branch no longer calls
`evaluate_canonical_normal_shadow(...TrackCruise)` or
`canonical_normal_control()` with a five-state selection. Existing five-state
helpers may remain temporarily for other intents or later physical deletion,
but they are unreachable as Track/Cruise publication owners.

# Design

## Causal audit

The earlier normal-fallback Slice deleted every runtime call which could solve or publish a
three-state normal command. The remaining graph is:

```text
LegacySpatialMpc3State
  -> enum + to_string + unreachable schema branch

ProgressContouring3State
  -> enum + to_string + unreachable schema branch + two rejection tests

convert_extended_solution_to_legacy()
  -> declaration + definition + two dedicated tests
```

There is no production producer or call site. These are not fallback safety mechanisms; they are
reconnection points to the dual-formulation architecture already removed from normal dispatch.

## Target formulation set

```text
Unresolved
VelocityProgress5State
VelocitySteeringProgress6State
SolverDerivedBypass
```

`VelocityProgress5State` remains the canonical Follow/Overtake/Rejoin formulation.
`VelocitySteeringProgress6State` remains the canonical Track/Cruise formulation.
`SolverDerivedBypass` remains an explicitly noncanonical exceptional representation and is used to
test fail-closed rejection without preserving a retired normal formulation.

## Deletion boundary

Delete only retired representation and conversion code. Keep all typed five-state extraction,
warm-start rebasing, progress bounds/costs, physical proof and publication contracts.

## New exceptional paths

None.

# Requirements

## Objective

Physically remove the retired five-state normal-authority implementation after
all normal intents and Overtake tactical admission have moved to
`VelocitySteeringProgress6State`.

## Root evidence

The production executable no longer links the five-state canonical plan,
retained revalidation, plan adapter or Follow async libraries, but CMake still
builds and installs them.  `VelocityProgress5State`, a five-state-only shadow
warm-start contract and unused five/three-state wall helpers also remain in
production headers/source.  They have no live caller and can be reconnected
without violating the current source gates.

## Invariants

- Six-state normal command, physical proof, retained proof and tactical Gate A
  behavior are unchanged.
- Emergency and Recovery stay outside normal authority.
- Generic stage geometry and exact physical trajectory types used by the
  six-state adapter are preserved.
- No parameter, timeout, lease, margin, weight, horizon or solver setting is
  changed.
- Generated result JSON is not staged.

## Exit criteria

- No five-state normal formulation enum, schema, library, source/header or
  controller helper remains in the package.
- No old canonical-plan/retained/follow migration target remains in CMake.
- Negative formulation tests use the live explicit noncanonical bypass.
- Build and all remaining package tests pass.
- Moving acceptance proves Track/Cruise and at least one Overtake entry still
  use the six-state authority, or explicitly records why positive Overtake
  evidence was not produced without restoring legacy code.

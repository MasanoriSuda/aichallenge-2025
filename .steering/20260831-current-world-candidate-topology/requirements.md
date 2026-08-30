# Requirements: current-world candidate topology

## Objective

Repair the candidate-generation defect classified by the frozen sequence-992
architecture comparison without changing production authority, solver settings,
clearance, Mission lifecycle or runtime configuration.

## Invariants

- candidate temporal topology is derived only from the current immutable world;
- a stateless seed carries no forced schedule from the captured candidate;
- every candidate retains the existing seven-state SQP and exact wall, timed
  obstacle, current-world and terminal successor proof chain;
- candidate population remains bounded to three per side;
- a finite target tube is not extrapolated beyond its sealed physical extent.

## Definition of Done

- reproduce `A/B fail, C succeeds` on the frozen snapshot;
- add regression tests for stale schedule deletion and finite encounter
  boundary generation;
- pass focused and package tests plus `make autoware-build`;
- run bounded `make dev3` validation and inspect candidate adoption/authority;
- document the invariant and experiment classification;
- commit only intended source, tests and documentation.

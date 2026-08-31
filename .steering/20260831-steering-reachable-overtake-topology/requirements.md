# Requirements: steering-reachable Overtake topology

## Objective

Repair the candidate-generation defect frozen at decision 1833 without
changing Mission lifecycle, production authority, solver settings, wall or
opponent clearance, timeouts, leases, retries or fallbacks.

## Frozen evidence

Source:

`output/20260831-124927/d1/mpcc_architecture_snapshots/000000001141-1f493d8bb2bb9e50-shiftout-side-negative-post-refinement-linearization-physical-dynamic-sqp-audit-solve-rejected/snapshot.yaml`

The unchanged A/B/C/D comparison found:

- persistent A: wall proof rejected;
- direct stateless B on both sides: wall proof rejected;
- rough left C: accepted at `(transition=6, ahead=20)`;
- offline left D: accepted at `(transition=6, ahead=20)`;
- physical diagonal left F: accepted when full-side is reached at stage 6;
- production G: no certified candidate; its only gradual member reaches
  full-side at fixed midpoint stage 9.

This is `A/B fail, C succeeds`: candidate generation, not Mission retention,
single-SQP, proof tolerance or physical infeasibility.

## Invariants

- derive the added temporal topology only from the immutable current-world
  request and physical steering/yaw-response limits;
- keep all existing seven-state SQP, wall, dynamic-obstacle and terminal Stop
  certificates unchanged;
- do not allow an uncertified candidate to reach normal authority;
- keep candidate evaluation asynchronous and bounded;
- preserve the already evidenced midpoint and late-exact topologies.

## Definition of Done

- the frozen source produces a steering-reachable full-side stage of 6;
- production comparison certifies the left sibling through that candidate;
- prior midpoint and late-exact candidate families remain represented;
- focused tests, package tests and `make autoware-build` pass;
- bounded dynamic validation shows no stale or uncertified publication;
- source, tests, design evidence and specification are committed together.

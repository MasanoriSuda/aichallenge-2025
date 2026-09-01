# Requirements

## Objective

Build one immutable temporal dataset contract that combines the prior admitted
teacher runs with the newly audited competition-failure prefixes, without
copying sequence identities or changing production runtime.

## Constraints

- source sequence split and run identity are immutable
- contact suffix is excluded at least one second before the first penalty or
  confirmed 0.5 m breach, whichever is earlier
- physical LiDAR remains metres and ego speed synchronization remains <= 50 ms
- the peer d3 failure remains validation-only
- no candidate is trained until the combined contract is verified

## Definition of Done

- repeated source roots are supported without filesystem aggregation tricks
- duplicate root or sequence identity is rejected before partial output
- combined recurrent dataset has complete provenance for all old/new sequences
- production checkpoint and runtime settings remain unchanged

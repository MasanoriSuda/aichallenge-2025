# Candidate5 peer gate requirements

## Objective

Determine whether the full-network candidate that learned the pre-contact
correction resolves the production four-peer contact trap without losing race
continuity.

## Frozen comparison

- world: deterministic `e2e-final`
- candidate: `f84e802fc6976906ddb062e1f5ddd509119a39602b55051b25e519b761c0f9e3`
- production baseline evidence: `output/20260901-085903`
- teacher ceiling evidence: `output/20260901-100204`
- control mode: production `fixed_lidar_brake`; no runtime teacher

## Definition of Done

1. All four launch logs prove the candidate checkpoint and production mode.
2. The run reaches Finish or yields a finalized failure snapshot.
3. Every finalized bag is checked for low-speed and positive-acceleration stall.
4. Candidate5 is promoted, rejected, or routed to a residual architecture from
   measured evidence; production remains unchanged during the gate.

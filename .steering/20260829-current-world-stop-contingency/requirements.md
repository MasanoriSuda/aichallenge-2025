# Current-world exact Stop contingency

## Objective

Restore bounded receding-horizon availability after commit `77cb25e1`
without reintroducing uncertified current-stage authority.

## Evidence

Run `output/20260829-111010` rejected D1 at decision 1034 with
`terminal-contingency-unavailable`. The normal continuation was statically
clear, but a future dynamic-obstacle intersection reduced proof scope to the
current stage. The vehicle then stopped because no exact Stop suffix existed.

## Required contract

For every partial retained publication, prove one causal maneuver:

1. apply the exact current command for one publisher interval;
2. apply the configured physical maximum braking with the last serialized
   steering held;
3. reach zero speed through the same nonlinear seven-state model;
4. prove the swept footprint against the immutable wall map and the current
   dynamic world;
5. bind the proof to the current decision and artifact identity.

The proof is rebuilt every control cycle. It is not a lease, timeout, fallback,
or retained geometric Mission.

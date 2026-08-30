# Audit

## Hypotheses

### H1: terminal Stop owns the wrong lateral reference

- Support: all left/pass-side primary trajectories pass normal proof and fail
  only after the common zero-offset Stop bends toward the racing line.
- Support: `ContingencyStopIntent::hold_lateral_m` is written by stateless
  maneuver construction but never consumed by Stop synthesis or publication.
- Refutation: the declared Mission offset and 128 uniformly distributed fixed
  lateral targets all reach the same wall collision or violate the physical
  lateral envelope.
- Status: refuted as the root cause.  The unused declaration is a real contract
  smell, but wiring it into production would not repair this failure.

### H2: current-world worker replacement causes the authority loss

- Support: replacement pressure existed in the first run.
- Refutation: preserving the pending immutable snapshot produced
  `replaced=0`, yet the same Stop/normal alternation and wall Recovery recurred
  in `output/20260830-200852`.
- Status: refuted; experiment removed.

### H3: no physical braking trajectory exists from this state

- Support: every existing A/B/C/D/G/H arm fails terminal wall proof.
- Refutation: a seven-state Stop solve reaches rest with exact wall and peer
  proof after the already-selected 25 ms publisher command is replayed.
- Status: refuted.

### H4: the fixed maximum-braking/fixed-lateral Stop candidate family is
structurally incomplete

- Support: zero offset, declared offset and the complete sampled fixed-offset
  family all fail.
- Support: the causal seven-state Stop solve succeeds from the exact next
  publisher boundary without changing acceleration, steering, wall, footprint
  or peer limits.
- Evidence: terminal velocity `-5.0012e-14 m/s`, exact minimum lateral reserve
  `0.293009 m`, exact dynamic proof clear.
- Status: confirmed.  This is a candidate-generation defect, not a tolerance,
  Mission-lifecycle or physical-infeasibility defect.

## Existing-patch relation

Commit `8dc45378` correctly unified the previously different terminal proof and
Emergency publisher under one racing-line tracking law. It fixed a real
model/publisher mismatch. The audit confirms that restoring separate Stop
implementations or merely consuming `hold_lateral_m` would be wrong.  The
single owner must remain, but its candidate must become a certified
seven-state braking trajectory instead of a fixed-policy path-feedback suffix.

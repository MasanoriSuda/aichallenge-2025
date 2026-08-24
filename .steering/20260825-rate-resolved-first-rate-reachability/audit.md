# Audit

## Causal finding

The rate-resolved QP anchored its state-zero steering equality to the semantic
current steering, but numerical equality residual can reconstruct that state
slightly inside the actuator box. The first steering-rate row was still the
generic `[-rate_max, rate_max]` box. An outward rate could therefore satisfy
the certified QP while the same rate integrated from the immutable semantic
steering crossed the physical steering limit at the 25 ms publication point.

The downstream sampler correctly exposed this mismatch. Relaxing or clamping
that sampler would have hidden a real reachability defect.

## Responsibility repair

- The persistent solver exposes its immutable physical absolute and relative
  row-tolerance contract. Row preconditioning remains unchanged.
- The semantic adapter derives the exact first-stage rate interval from the
  semantic steering, steering limit, rate limit and immutable stage duration.
- The adapter moves that interval inward by a margin that covers every
  solver-certified row residual. An empty certified interior fails closed.
- Later-stage rate boxes are unchanged because only the first stage can be
  published before the next receding-horizon solve.
- The publication sampler remains the final strict check; no output clamp was
  introduced.

## Authority audit

The modified formulation is still connected only to the observation-only
Track/Cruise rate-resolved shadow. It cannot write a plan store, select normal
authority, seed production warm state or publish a control command. The
aggregate trace now exposes physical bounds, solver bounds and certificate
margin with `authority=shadow, selected=0`.

All 24 single-authority source-contract tests pass.

## Static conclusion

The implementation repairs the upstream physical reachability contract rather
than suppressing the observed sample rejection. No parameter, solver setting,
fallback, timeout or production authority changed. Dynamic shadow evidence is
still required before this Slice can be accepted.

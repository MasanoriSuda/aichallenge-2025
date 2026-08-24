# Audit

## Root-cause boundary

The previous dynamic run proved that the semantic adapter, QP assembly and
six-state OSQP solve succeeded for every consumed request. The first opaque
failure was the executable 40 Hz sample boundary. This Slice therefore does
not change the formulation, bounds or solver; it makes that boundary typed.

## Behavioral-equivalence audit

- `evaluate_actuation_sample()` evaluates the same predicates, in the same
  order, with the same `1e-12` tolerance as the previous optional validator.
- `sample_actuation()` remains the compatibility API and returns only the new
  evaluator's optional sample.
- No rejected value is clamped, rounded or converted to a fallback.
- Deterministic tests cover time, initial steering, steering-rate, terminal
  steering and non-finite rejection, plus the accepted compatibility path.

## Authority audit

The typed result flows only through:

```text
rate-resolved shadow solver
  -> observation-only result mailbox
  -> aggregate diagnostic log
```

Repository search finds no use in plan selection, authority admission,
solution history, final steering or command publication. The single-authority
source-contract suite also passed all 24 tests.

## Static conclusion

The instrumentation is eligible for a `make dev2` evidence run. It is not a
production-authority promotion and provides no justification for changing a
bound or tolerance until the runtime reason distribution is known.

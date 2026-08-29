# Audit: Stop-to-normal authority deadlock

## Observed phenomenon

Run `output/20260829-214906` dynamically accepted the new Cruise/Follow side
population, then both domains eventually stopped.  The repeated terminal
decision was:

```text
previous=stop, proposed=cruise, effective=stop
resolution=previous-retained
gate_a_attempted=0
proposed_world=cursor-unavailable|steering-unreachable|progress-lift-rejected
```

The visible permanent Stop is downstream.  Its upstream producer is the
current-world join which refuses an otherwise physically proved connection.

## Hypotheses and falsifiers

1. **Physical infeasibility** — falsified for the dominant D1 population by
   5,978 accepted exact current-world connection proofs.  A failed proof still
   remains fail-closed.
2. **Missing Cruise side candidates** — falsified by accepted positive and
   negative candidates and D2's 38 exact-clear dynamic results.
3. **Stop retention rule alone** — insufficient explanation.  Retaining a
   proved Stop is correct while no successor exists; the defect is that a
   proved successor is deliberately prevented from existing.
4. **Persistent publication join defect** — confirmed.  Fresh candidates are
   evaluated from a new wire predecessor, the bounded feedback connector and
   exact proof accept, then `complete_continuation_proof()` overwrites success
   with `SteeringUnreachable` solely because `feedback_shadow_mode` is true.

## Existing patch relationship

The 2026-08-27 atomic Stop retention fixed an unsafe authority gap: it made the
actually published Stop visible.  It did not create this physical mismatch,
but it makes the upstream connector defect permanent instead of oscillating
through an uncertified legacy command.  That patch remains valid.

The rejected full latest-state feedback QP changed only x0 in an old-origin
problem and was correctly removed.  This Slice does not restore it.  It uses
the already measured current-state nonlinear continuation and unchanged exact
proof as a stateless bundle.

## Implemented repair

- A completed latest-state connection now constructs the common retained
  `Proof` and is explicitly tagged `latest_state_feedback_bundle`.
- The production adapter remains the only normal-authority adapter and uses
  the proved current-world actuation and continuation.
- The final publication ledger does not call `Store::mark_executed()` for a
  stateless bundle, because the source artifact's first command was not what
  crossed the wire.
- A failed continuation, wall, dynamic, Follow or terminal proof still
  restores `SteeringUnreachable` and creates no `Proof`.

No parameter, solver setting, clearance, timeout, lease, grace, retry,
fallback or additional normal publisher was added.  The deleted production
assumption is the diagnostic-only discard of an otherwise complete proof.

## Verification

- Source authority contract: 75/75 passed.
- Focused retained/production tests: 2/2 CTest targets passed.
- Full package suite: 54/54 CTest targets passed.
- `make autoware-build`: 25 packages passed.
- `git diff --check`: passed before documentation update.

Dynamic Gate `output/20260829-220933` used `make dev2` against the frozen
baseline run `output/20260829-214906`:

| D1 metric | Baseline | Bundle Slice |
|---|---:|---:|
| logged normal decisions | 118 | 406 |
| logged Emergency decisions | 175 | 412 |
| maximum Emergency streak | 16 | 9 |
| current-world Bundle decisions | 0 | 121 |
| logged Emergency -> Bundle exits | 0 | 114 |
| final / maximum logged speed | 0.00 / 0.75 m/s | -0.33 / 7.95 m/s |

The negative final speed is a later independent stuck-Recovery episode at the
manual end of the run; before it, the Bundle repeatedly restored normal
authority and reached 7.95 m/s.  The run therefore accepts the repaired
Stop-to-normal invariant but does not pass the broader integration-quality
Gate.  Short Emergency insertions, callback overrun windows and Recovery
overrides remain separate frozen evidence for the next audit.

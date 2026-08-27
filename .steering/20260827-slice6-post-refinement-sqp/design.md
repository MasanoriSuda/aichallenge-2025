# Design

## Earliest violated invariant

The trajectory optimized by the final QP and the trajectory replayed for the
physical certificate must represent the same local nonlinear dynamics.

## Causal chain

```text
broad semantic QP
  -> one nonlinear relinearization and solve
  -> wall/dynamic refinements change the final primal
  -> no relinearization after those refinements
  -> affine QP trajectory satisfies its rows
  -> exact nonlinear replay leaves the current lateral box
  -> fresh physical certificate withheld
  -> previous artifact becomes stale
  -> retained current-stage replay also fails
  -> explicit Emergency, Mission loss and Recovery
```

The visible wall abort is downstream.  Relaxing clearance or retaining an
unproved command would hide the formulation mismatch.

## Repair

After all requested wall and dynamic-obstacle refinements have been assembled
and solved:

1. build a provisional immutable artifact from the final refined primal and
   run the exact physical replay which production uses;
2. if that replay already succeeds, publish it without another QP solve;
3. only when exact replay rejects the trajectory, relinearize the temporal
   Frenet dynamics around that same final refined primal;
4. keep every existing cost, state/input box, swept-wall row,
   progress-aligned wall row and dynamic-obstacle row;
5. build a bootstrap from that corrected current problem rather than reusing
   primal/dual values belonging to the replaced equality rows;
6. solve and recheck the same physical problem, with a named deterministic
   upper bound of three SQP corrections;
7. construct the production artifact only from a solution which passes the
   exact replay; otherwise return the typed `physical-proof-rejected` outcome.

This is a proof-driven final SQP certification of one canonical formulation,
not a second controller or retry fallback.  The proof gate is intentionally
checked before requesting a correction: the first unconditional-correction
prototype created new OSQP maximum-iteration failures even when the refined
trajectory was already physically valid.

## Rejected alternatives

- Increase physical tolerance: hides nonlinear model error in the proof.
- Extend retained age: publishes a command after current-stage proof failed.
- Reuse stale primal/dual after changing equalities: repeats the already
  repaired provenance defect.
- Add another normal fallback: violates Slice 6 single authority.

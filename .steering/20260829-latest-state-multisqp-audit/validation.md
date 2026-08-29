# Validation

## Result

D does not rescue the frozen extreme counterexample and therefore must not be
promoted as a runtime retry.

The deterministic 20-stage case produces this causal sequence:

```text
B: direct retained suffix
   -> first QP rejected

C: reachable nonlinear candidate + one SQP
   -> QP solved
   -> exact nonlinear physical proof rejected

D: same immutable C problem + bounded multi-SQP
   -> solve #1 succeeds, exact proof rejects
   -> relinearize the same problem around solve #1
   -> solve attempt #2 is primal infeasible
```

The result is not evidence for a larger OSQP iteration limit, looser physical
tolerance or another runtime fallback. It shows that the candidate-generation
defect exposed by B is real, while this particular extreme state remains
outside what C/D can certify under the frozen physical problem.

This is classified as physical infeasibility for the synthetic frozen case,
with one important qualification: a live failure may have different current
world geometry. Live classification therefore requires an immutable snapshot
captured at the failure and replayed offline; it does not justify connecting D
to production authority.

## Invariants checked

- D with iteration limit one produces the same accepted candidate states as C.
- The correction loop changes only temporal Frenet linearizations.
- References, costs, state/input boxes, wall rows, swept-wall rows,
  dynamic-obstacle rows, identity and terminal semantics remain in the same
  `AssemblyRequest`.
- A new primal bootstrap is rebuilt under each new affine equality system.
- Only `ExactTrajectoryRejected` may request another SQP correction;
  structural physical-adapter errors terminate immediately.
- Audit requests are explicitly bounded to eight attempts.
- The evaluator has no Store, mailbox, command candidate or publisher API.

## Commands

- focused reachable-bridge tests: 5/5 passed;
- `make autoware-build`: 25 packages passed;
- package regression: 54/54 test targets passed;
- `git diff --check`: passed.

## Live-observation decision

Live evidence is justified, but an online D solver is not. The next Slice must
capture one immutable latest-state feedback problem at the normal-authority
failure boundary and replay A/B/C/D outside the command path. Capture must not
add a producer, retry, lease, timeout, solver setting or clearance change.

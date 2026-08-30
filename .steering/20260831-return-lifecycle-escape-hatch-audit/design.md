# Design: Return lifecycle escape-hatch audit

Keep the production publisher, Store, mailbox and authority graph untouched.
Extend only the offline architecture comparison so Return can be evaluated
with independent current-world rejoin geometry.

The four required arms are:

- A: captured persistent Return geometry plus current seven-state SQP;
- B: the existing current-world Return rebuild plus the same SQP;
- C: bounded smooth rejoin schedules generated independently from the current
  state to the immutable terminal Return contract, then seven-state refined;
- D: the same independent schedules with bounded offline SQP continuation.

Return does not use pass-side disjunction schedules.  Its C/D search dimension
is the rejoin start/completion schedule.  Every candidate retains the same
target/homotopy identity, wall grid, obstacle predictions, terminal contract
and source interaction fingerprint.  Only the trajectory/reference schedule
is regenerated.

No result is accepted without the existing exact physical and terminal Stop
proof pipeline.  If all sampled C/D schedules fail, the result remains bounded
evidence only; physical infeasibility requires sufficient candidate coverage
or an explicit nonlinear infeasibility proof.

# Design: dense-wall solver-owner A/B

## Why this comparison

The dense nonlinear interior-wall oracle is the only tested formulation that
closes the frozen Follow proof. It has 478 added rows and solves in about
44 ms, but the same formulation reaches 4000 iterations for ShiftOut and
Cruise. A four-row structured replacement and proof-guided cuts both failed.

OSQP treats scaling as setup-time solver state. The controller already owns a
separate internally equilibrated workspace for the wall-refinement QP class,
but the latest-state dense audit currently uses the ordinary normalized
workspace. Therefore the next comparison changes only the numerical owner.

## Isolation

`LatestStateFeedbackSolverContext` receives a second private solver workspace.
The equilibrated entry point selects it before the first solve. It does not
run ordinary OSQP first, consume an ordinary result, or expose a Store or
publisher API.

Both arms assemble the same dense problem and invoke the same artifact builder
and exact physical adapter. A successful solve that fails proof remains a
model/certificate mismatch, not an accepted result.

## Exit classification

- A fails, B passes exact proof: numerical owner/conditioning defect;
- A/B pass: equilibration is unnecessary for that snapshot;
- A passes, B fails: equilibration regression;
- A/B fail before proof: OSQP-local ownership remains unresolved;
- solve succeeds but proof fails: formulation/certificate mismatch remains.

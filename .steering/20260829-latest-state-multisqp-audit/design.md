# Design

```text
C reachable rollout
  -> linearize unchanged QP
  -> solve #1
  -> exact nonlinear proof
       accepted: stop
       rejected: relinearize same QP around solve #1
                 -> solve #2 -> proof
                 -> ... explicit offline limit
```

The loop is an audit of the single-SQP approximation, not a runtime fallback.
No iteration changes references, costs, boxes, wall rows, obstacle rows,
identity or terminal semantics. Dual state stays internal to the dedicated
observation solver; each new primal bootstrap is rebuilt under the current
affine equality rows.

Only an exact-trajectory proof rejection is eligible for another correction.
Identity, intent or stage-geometry rejection is structural and returns
immediately. The public audit API also rejects zero or more than eight
iterations so a caller cannot turn the diagnostic into an unbounded retry.

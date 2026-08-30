# Design

```text
normal LatestOnlyWorker
  -> solve selected current-world candidate
  -> exact wall + all-peer proof
  -> existing Store replacement (unchanged)
  -> submit {exact selected snapshot, exact normal artifact}
       to terminal Stop LatestOnlyWorker

terminal Stop LatestOnlyWorker
  -> publisher-boundary maximum-braking rebase
  -> bounded steering-rate lattice
  -> private seven-state SQP
  -> exact physical/wall/all-peer proof
  -> observation-only mailbox
```

The `RateResolvedPipelineEvaluation` carries the exact solver-source snapshot.
This is necessary because an Overtake population candidate may change side,
fingerprint and stage constraints relative to the original callback draft.
Reusing the draft would create a model/certificate mismatch.

The terminal worker is distinct from the normal worker.  Store admission
therefore completes before potentially expensive lattice enumeration.  The
control callback only consumes a completed observation record; it never waits
for the terminal worker.

The Stop artifact intentionally remains observation-only because its current
seven-state identity inherits the normal intent.  Production promotion needs
an explicit immutable companion identity and semantic successor contract,
especially for Return.

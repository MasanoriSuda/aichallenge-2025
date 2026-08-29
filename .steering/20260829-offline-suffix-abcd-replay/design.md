# Design

The existing v2 snapshot owns semantic request, exact refined QP, warm start,
wall grid, dynamic obstacles and world fingerprint. The replay tool derives a
time probe from that immutable data and evaluates four isolated
`LatestStateFeedbackSolverContext` instances so persistent OSQP state cannot
cross-contaminate arms.

```text
recorded v2 interaction snapshot
  + explicit elapsed time
  + interpolated diagnostic latest state/input
      -> A old-origin x0 replacement
      -> B common-clock suffix
      -> C reachable nonlinear suffix candidate
      -> D bounded multi-SQP of the same C problem
```

The interpolated state is a deterministic offline probe, not evidence that it
equals the vehicle's historical measured state. Results classify formulation
behavior for that probe only. A future exact live-state capture may replace
the interpolation without changing any solver arm.

CLI success means the comparison ran. A rejected solve or physical proof is a
valid comparison result and therefore does not make the process exit nonzero.

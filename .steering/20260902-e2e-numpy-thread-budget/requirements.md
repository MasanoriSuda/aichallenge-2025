# Requirements

## Objective

Make TinyLidarNet NumPy inference CPU ownership deterministic so small
current-sample matrix operations cannot oversubscribe the host and create
freshness-breaking latency tails.

## Root-cause evidence

- Runtime NumPy links OpenBLAS and reports 12 effective BLAS threads when no
  thread environment is set.
- The peer-512 GRU's representative current-sample inference benchmark:
  - default: mean `0.257 ms`, p95 `0.367 ms`, p99 `2.135 ms`, max `5.355 ms`;
  - one thread: mean `0.164 ms`, p95 `0.191 ms`, p99 `0.255 ms`, max
    `0.303 ms`.
- The rejected bounded-authority run had one shared speed-admission loss in an
  interval containing a `98.70 ms` full callback inference tail.

The isolated microbenchmark does not prove BLAS oversubscription caused that
exact race-cycle miss.  It does prove that 12-thread execution is slower and
far less deterministic for this model's small batch-one matrices.

## Constraints

- Set the BLAS budget only for the TinyLidarNet controller process.
- Do not change simulator, MPC peer or container-wide thread settings.
- Do not change checkpoints, model dimensions, control parameters, authority,
  speed freshness or any acceptance threshold.
- Production recurrent authority remains false.
- Verify single and three-vehicle authority-disabled shadow Gates before any
  later authority experiment.

## Definition of Done

- Launch contract fixes `OPENBLAS_NUM_THREADS=1` on the TinyLidarNet node.
- Startup evidence reports the executed thread-budget environment.
- Interface and controller tests pass.
- Single and three-vehicle Gates pass without penalty, stall, hidden reset,
  recurrent error or scan-rate regression.
- Timing is compared with the frozen pre-budget runs.

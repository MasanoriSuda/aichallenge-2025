# Design

## Process boundary

The production node continues to build the immutable recurrent sample from the
already-admitted Conv5/spatial result and publishes its production command
before submitting diagnostic work.

A bounded parent scheduler owns one running request and one replaceable pending
request.  Its worker is a standalone Python subprocess started with an
environment containing `OPENBLAS_NUM_THREADS=1`.  The worker imports NumPy only
after that environment exists, loads and verifies the exact recurrent artifact,
and returns plain immutable evaluations over a framed local pipe.  The bounded
parent scheduler owns hidden state and passes an explicit snapshot and successor
across that pipe, so generation resets remain atomic with latest-wins scheduling.

The ROS process never executes recurrent matrix operations.  The scheduling
thread may block on the private worker pipe, but the scan callback only updates
the bounded pending slot and drains completed results.

## Lifecycle

- Every request carries `sequence` and `generation`.
- A reset increments generation and drops pending work.
- The child resets hidden state before evaluating the first request of a new
  generation.
- Results from an older parent generation are classified
  `reset-superseded` and never update telemetry state.
- Age is measured from parent submission through child completion.
- Broken pipes, malformed replies and child inference errors increment explicit
  diagnostic errors; they never publish a command or trigger production Stop.

## Authority boundary

This Slice promotes only observation.  The production node no longer binds the
in-process evaluator to its observation scheduler.  The pre-existing
synchronous authority experiment remains unreachable while authority is false;
a later authority Slice must either remove or replace it with a fresh,
sequence-matched merge contract and pass its own dynamic Gates.  It may not
publish delayed shadow results.

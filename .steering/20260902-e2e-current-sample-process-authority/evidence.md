# Evidence

## Static acceptance

- Host focused recurrent/core suite: `53 passed`.
- Host full TinyLidarNet controller suite: `92 passed`.
- Host E2E launch contract: `3 passed`.
- `make autoware-build`: 25 packages completed successfully.
- Docker installed-space TinyLidarNet suite: `92 passed`.
- Docker installed-space E2E launch contract: `3 passed`.
- Python compilation and `git diff --check`: passed.

The core no longer evaluates recurrent steering authority through its local
NumPy model merely because the authority flag is true.  An explicitly bound
current-sample evaluator is mandatory.  Without it, the recurrent result is
classified as an inference error and the already-valid spatial production
command is returned without recurrent authority.

The production ROS binding starts an exact-SHA subprocess, verifies the
self-described runtime contract, constrains child OpenBLAS to one worker and
binds that evaluator before subscriptions start.  The child reply must match
the current private request sequence.  Authority-disabled observation retains
the accepted asynchronous latest-wins path.

## Dynamic acceptance

### Single vehicle: pass

Run: `output/20260902-e2e-current-sample-authority-single-b`

- Finished 3/3 laps with `84.6131 / 84.8830 / 84.1084 s` and
  `253.6045 s` total.
- Penalties: zero; stall analysis: pass.
- Exact recurrent artifact SHA:
  `b4b292e0223444c84bf85523d31d2c475386e7743416fc9d4eaff31dc7243830`.
- Authority applied: `7851 / 7851`; coverage: `1.0`.
- Recurrent skip/error/stale/reset: all zero.
- Recurrent inference: weighted mean `6.6432 ms`, maximum `38.10 ms`;
  minimum scan rate `19.8 Hz`.

The accepted authority-disabled process-shadow single baseline was
`252.2603 s`.  B was `1.3442 s` (`0.53%`) slower, so the clean single run proves
the process/current-sample contract but not a performance benefit.

### Three vehicle: reject

Run: `output/20260902-e2e-current-sample-authority-peer-b`, evaluated on d3.

- Finished 3/3 laps, first of three, with
  `85.2328 / 84.2333 / 84.1884 s` and `253.6545 s` total.
- Penalties: zero; stall analysis: pass.
- Authority applied: `9120 / 9152`; coverage: `0.99650`.
- Recurrent skips: `32`; hidden resets: `32`; non-ok intervals: `1`.
- Recurrent inference: weighted mean `20.9503 ms`, maximum `96.58 ms`;
  minimum scan rate `19.57 Hz`.
- Strict recurrent Gate rejected the run for an inference/non-ok interval,
  reset threshold violation and skipped authority inference.

The accepted authority-disabled process-shadow peer baseline was
`253.7645 s`.  B was only `0.1100 s` (`0.04%`) faster, which is immaterial and
does not compensate for losing the temporal-state/current-sample contract.

## Decision

Reject process-synchronous current-sample recurrent authority for production.
The experiment isolates the remaining defect: synchronous rendezvous is clean
in a single-vehicle process budget but is not schedulable under the three-
vehicle workload.  The model weights, correction bound and race parameters
must not be retuned to hide this execution-contract failure.

Keep the accepted authority-disabled process shadow, packaged authority default
`false`, spatial checkpoint and production parameters unchanged.  Any future
recurrent-authority Slice must change the execution architecture (for example,
move the recurrent policy into a canonical fixed-rate controller process with
an explicit input admission contract), not add a delayed-result merge or retry
this configuration with looser Gates.

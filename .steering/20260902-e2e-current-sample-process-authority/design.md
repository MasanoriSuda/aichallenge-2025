# Design

## Canonical authority path

The core still constructs the recurrent sample only after the frozen Conv5 and
spatial production result have passed their current-sample admission.  For
explicit recurrent authority it no longer evaluates the recurrent NumPy model
locally.  The ROS node must bind one callable evaluator before callbacks start.

The production binding is `RecurrentShadowSubprocessEvaluator`:

1. send immutable Conv5/spatial sample, current hidden state and request
   sequence;
2. child evaluates with `OPENBLAS_NUM_THREADS=1`;
3. parent accepts only the exact sequence reply;
4. core records the successor hidden state and applies the configured bounded
   correction before returning the current command;
5. longitudinal safety and speed governance retain their existing ownership.

There is no asynchronous merge.  Authority-disabled observation continues to
use the accepted process-async latest-wins executor after publishing the
production command.

## Deadline and failure semantics

The authority response timeout uses the already-declared watchdog period
(`0.05 s` in the frozen launch).  A timeout can leave a late reply on the pipe,
so the evaluator closes the private channel and cannot let that reply satisfy a
later request.  The core classifies the current recurrent evaluation as an
inference error, clears temporal state, and publishes the spatial production
command.  It does not publish Stop and does not restart or retain authority.

Worker initialization failure is different: an explicitly requested authority
configuration cannot start without its exact worker identity, so node startup
fails closed.  Authority-disabled observation failure remains diagnostic-only
and leaves production running.

## Acceptance order

Use the accepted authority-disabled process runs as A.  Execute B with the same
peer-512 artifact and the existing `+/-0.24 rad` correction bound.  Pass the
single Gate before starting the domain-3 peer Gate.  A timing-clean run without
material performance gain can validate the architecture but does not by itself
authorize packaging or changing the default.
